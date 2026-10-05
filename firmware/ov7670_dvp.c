// Guarded as a whole: DCMI_IRQHandler here overrides the startup vector table's
// weak alias, so this object is linked unconditionally. Without the guard the
// capture ring is allocated even when NCAP_DVP=0, which overflows RAM on a
// 320x240 synthetic build.
#if NCAP_DVP

#include "ov7670_dvp.h"

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "i2c1.h"
#include "ov7670.h"

// ---- DCMI / DMA2 tunables ----

#ifndef DVP_DMA_STREAM
#define DVP_DMA_STREAM 1
#endif
#ifndef DVP_DMA_CHANNEL
#define DVP_DMA_CHANNEL DMA_CHANNEL_1
#endif

// DCMI_DR -> DMA2 Stream1 Channel 1 (RM0090 DMA2 request mapping). The
// platform wires dcmi DMAReceive -> dma2@1.
// The F4 HAL only defines GPIO_PIN_0..15 individually; DCMI_HREF is PA17.
#define DVP_PIN_HREF  (1UL << 17)

static DMA_HandleTypeDef hdma_dcmi;

// The capture buffer doubles as the de-interleaved luma frame: chroma is
// compacted out in place. A separate luma buffer would not fit - at
// VIDEO_FRAMES*YUV_FRAME_BYTES plus another luma copy the total exceeds the
// F446's 128 KB. In-place is safe because the destination index never exceeds
// the source index (dst i <= src 2i for YUV422, dst 4k+j <= src 6k+j for 420).
static uint8_t dvp_ring[DVP_RING_BYTES] __attribute__((aligned(4)));

static volatile bool dvp_frame_ready;
static volatile uint32_t dvp_ready_slot;
static volatile uint32_t dvp_frames;
static volatile uint32_t dvp_accepted;
static volatile uint32_t dvp_rejected;
#if DVP_ALIGN_CHECK
// Row 0 of the last accepted frame, used as the alignment reference.
static uint8_t dvp_ref_row0[VIDEO_WIDTH];
static bool dvp_have_ref;
#endif
static volatile uint32_t dvp_overruns;
static volatile uint32_t dvp_write_slot;

// Byte offset of the frame currently being written / last written.
// Poll-only mode skips the DCMI interrupt entirely and relies on the RISR
// poll in ov7670_dvp_take_frame(). Useful when the NVIC wiring for DCMI is
// suspect, and it removes interrupt-storm risk on a starved link.
#ifndef DVP_POLL_ONLY
#define DVP_POLL_ONLY 0
#endif

static uint32_t dvp_slot_bytes(void)
{
    return DVP_FRAME_BYTES;
}

// Words transferred per frame slot; YUV frame sizes are always a multiple of 4
// for the geometries this supports.
static uint32_t dvp_slot_words(void)
{
    return DVP_FRAME_BYTES / 4u;
}

static inline uint32_t dvp_dma_ndtr(void)
{
    return DMA2_Stream1->NDTR;
}

static inline void dvp_dma_stop(void)
{
    __HAL_DMA_DISABLE(&hdma_dcmi);
    DCMI->CR &= ~DCMI_CR_CAPTURE;
}

static inline void dvp_dma_start(void)
{
    __HAL_DMA_DISABLE(&hdma_dcmi);
    hdma_dcmi.Instance->PAR = (uint32_t)&DCMI->DR;
    hdma_dcmi.Instance->M0AR = (uint32_t)(dvp_ring + (dvp_write_slot * dvp_slot_bytes()));
    // DCMI packs 4 bytes into DR per DMA request, so NDTR counts words.
    hdma_dcmi.Instance->NDTR = dvp_slot_words();
    __HAL_DMA_ENABLE(&hdma_dcmi);
    DCMI->CR |= DCMI_CR_CAPTURE;
}

// ---- de-interleave ----

static void dvp_strip_in_place(uint8_t *buf)
{
#if DVP_LUMA_ONLY
    // The DCMI already dropped the chroma bytes; the buffer is luma as it lands.
    (void)buf;
#else
    // YUYV: Y0 Cb Y1 Cr Y2 Cb Y3 Cr ... keep the even bytes.
    const uint32_t n = (uint32_t)VIDEO_WIDTH * DVP_STREAM_H;
    for (uint32_t i = 0; i < n; i++)
    {
        buf[i] = buf[i * 2];
    }
#endif
}

// ---- pin configuration ----

// STM32F446 DCMI is AF13 with no remap except DCMI_D0 (PC6 or PC13):
//   PIXCLK PA6, VSYNC PA4, HREF PA17, PWDN PA11,
//   D0..D5 PC6..PC11, D6 PC12, D7 PD6
// PA4/PA6 are why the W5500 had to move off SPI1.
static void dvp_gpio_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF13_DCMI;

    g.Pin = GPIO_PIN_6;                    // PIXCLK (input direction)
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_4 | DVP_PIN_HREF;    // VSYNC, HREF
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 |
            GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    HAL_GPIO_Init(GPIOC, &g);

    g.Pin = GPIO_PIN_6;                    // D7
    HAL_GPIO_Init(GPIOD, &g);

    // PWDN low = power down; drive it high to run.
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Alternate = 0;
    g.Pin = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOA, &g);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
}

// ---- init ----

bool ov7670_dvp_init(void)
{
    // SCCB still has to configure the sensor: COM7 selects QVGA and YUV422,
    // CLKRC the pixel-clock divider, etc. Only PIXELS come from the parallel
    // bus - control still goes over SCCB.
    if (!ov7670_init())
    {
        printf("F4: dvp: ov7670 SCCB init failed\r\n");
        return false;
    }

    dvp_gpio_init();
    __HAL_RCC_DCMI_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    __HAL_DMA_DISABLE(&hdma_dcmi);
    hdma_dcmi.Instance = DMA2_Stream1;
    hdma_dcmi.Init.Channel = DVP_DMA_CHANNEL;
    hdma_dcmi.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_dcmi.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dcmi.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dcmi.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_dcmi.Init.Mode = DMA_NORMAL;
    hdma_dcmi.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_dcmi.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_dcmi) != HAL_OK)
    {
        printf("F4: dvp dma init failed\r\n");
        return false;
    }

    // Hardware sync, active-high VSYNC, active-low HREF, rising-edge PCLK,
    // 8-bit extended data mode, continuous capture.
    uint32_t cr = DCMI_CR_ENABLE
                | DCMI_CR_ESS
                | DCMI_CR_VSPOL
                | DCMI_CR_PCKPOL
                | 0u;   // EDM=00: 8-bit capture on D[7:0]
#if DVP_CROP_ROWS > 0
    // Crop the captured window to the top DVP_CROP_ROWS lines. CWSTRT is
    // (X0 | Y0<<16) and CWSIZE is (width | height<<16), both 14-bit fields,
    // and the width is in PIXELS (the model scales it to bus bytes).
    DCMI->CWSTRTR = 0u;
    DCMI->CWSIZER = (uint32_t)VIDEO_WIDTH | ((uint32_t)DVP_CROP_ROWS << 16);
    cr |= DCMI_CR_CROP;
    // Snapshot mode: stop at the frame end so the re-arm below lands on the
    // next frame start rather than wherever the pixel clock has got to.
    cr |= DCMI_CR_CM;
#endif

#if DVP_LUMA_ONLY || NCAP_DCMI_TRY_BSM
    // Byte Select Mode. UNVERIFIED on F4: RM0090 marks CR bits 31:15 reserved
    // while stm32f446xx.h defines BSM_0/BSM_1 at 16/17 and OEBS at 18. BSM=01
    // (OTHER) with OEBS=0 keeps every other byte starting at the first, which
    // for YUYV leaves exactly the luma bytes - so chroma never enters RAM and a
    // frame costs 1 byte per pixel instead of 2. If the silicon ignores these
    // reserved bits the stream is simply byte-aligned garbage.
    cr |= DCMI_CR_BSM_0 | DCMI_CR_OEBS;
#endif
    DCMI->CR = cr;

    DCMI->ICR = DCMI_ICR_FRAME_ISC | DCMI_ICR_OVF_ISC | DCMI_ICR_VSYNC_ISC |
                 DCMI_ICR_LINE_ISC | DCMI_ICR_ERR_ISC;
    DCMI->IER = DCMI_IER_FRAME_IE | DCMI_IER_OVF_IE;

#if !DVP_POLL_ONLY
    HAL_NVIC_SetPriority(DCMI_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DCMI_IRQn);
#endif

    printf("F4: dvp ring %u B (%u frames x %u B, %s), luma/frame %u B\r\n",
           (unsigned)DVP_RING_BYTES, (unsigned)DVP_FRAMES,
           (unsigned)DVP_FRAME_BYTES, DVP_LUMA_ONLY ? "luma-only (bsm)" : "yuv422",
           (unsigned)DVP_LUMA_FRAME_BYTES);

    dvp_frame_ready = false;
    dvp_ready_slot = 0;
    dvp_frames = 0;
    dvp_accepted = 0;
    dvp_rejected = 0;
#if DVP_ALIGN_CHECK
    dvp_have_ref = false;
#endif
    dvp_write_slot = 0;
    dvp_dma_start();
    printf("F4: dvp CR=%08X SR=%08X IER=%08X\r\n",
           (unsigned)DCMI->CR, (unsigned)DCMI->SR, (unsigned)DCMI->IER);
    return true;
}

// Runs at the frame boundary: the slot the DMA just filled is complete, and the
// next slot is armed IMMEDIATELY. Deferring the re-arm to the main loop loses
// alignment whenever a frame slot is smaller than a sensor frame (DCMI crop),
// because the sensor keeps streaming while the firmware is busy, so the DMA
// would restart mid-frame and straddle two frames.
static void dvp_publish_and_rearm(void)
{
    dvp_ready_slot = dvp_write_slot;
    dvp_write_slot = (dvp_write_slot + 1u) % DVP_FRAMES;
    dvp_dma_stop();
    dvp_dma_start();
    dvp_frame_ready = true;
}

void DCMI_IRQHandler(void)
{
    uint32_t mis = DCMI->MISR;
    if (mis & DCMI_IER_OVF_IE)
    {
        DCMI->ICR = DCMI_ICR_OVF_ISC;
        dvp_overruns++;
    }
    if (mis & DCMI_IER_FRAME_IE)
    {
        DCMI->ICR = DCMI_ICR_FRAME_ISC;
        dvp_publish_and_rearm();
    }
}

bool ov7670_dvp_frame_ready(void)
{
    return dvp_frame_ready;
}

bool ov7670_dvp_take_frame(const uint8_t **luma)
{
    // Poll RISR as well as servicing the IRQ. A missed frame interrupt would
    // otherwise stall the stream silently, and polling also lets this work on
    // a board whose NVIC wiring for DCMI is not set up.
    // Poll fallback: with the interrupt disabled the boundary is seen late,
    // which is fine for a full-frame slot but cannot hold alignment under crop.
    if (!dvp_frame_ready && (DCMI->RISR & DCMI_RISR_FRAME_RIS) != 0u)
    {
        DCMI->ICR = DCMI_ICR_FRAME_ISC;
        dvp_publish_and_rearm();
    }
    if (!dvp_frame_ready)
    {
        return false;
    }
    dvp_frame_ready = false;

    const uint32_t slot = dvp_slot_bytes();
    uint8_t *frame = dvp_ring + ((dvp_ready_slot * slot) % DVP_RING_BYTES);
    dvp_strip_in_place(frame);
    dvp_frames++;

#if DVP_ALIGN_CHECK
    // Row 0 is now luma. Reject the frame if it looks like it was captured from
    // somewhere other than the top of the sensor frame.
    if (dvp_have_ref)
    {
        uint32_t sad = 0;
        for (uint32_t i = 0; i < VIDEO_WIDTH; i++)
        {
            int d = (int)frame[i] - (int)dvp_ref_row0[i];
            sad += (uint32_t)(d < 0 ? -d : d);
        }
        if (sad > (uint32_t)VIDEO_WIDTH * DVP_ALIGN_TOLERANCE)
        {
            dvp_rejected++;
            // Keep the old reference so a run of bad frames cannot poison it.
            return false;
        }
    }
    memcpy(dvp_ref_row0, frame, VIDEO_WIDTH);
    dvp_have_ref = true;
#endif
    dvp_accepted++;

    *luma = frame;
    return true;
}

uint32_t ov7670_dvp_frames_captured(void) { return dvp_frames; }
uint32_t ov7670_dvp_frames_accepted(void) { return dvp_accepted; }
uint32_t ov7670_dvp_frames_rejected(void) { return dvp_rejected; }
uint32_t ov7670_dvp_overruns(void)        { return dvp_overruns; }

#endif /* NCAP_DVP */
