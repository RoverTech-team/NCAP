#ifndef OV7670_DVP_H
#define OV7670_DVP_H

#include <stdbool.h>
#include <stdint.h>

// DCMI (parallel DVP) capture path for the OV7670.
//
// Pixel transport is the DVP bus ONLY (PCLK/VSYNC/HREF/D[7:0] -> DCMI -> DMA2).
// The OV7670 cannot deliver pixels over SCCB: the 0xF0-0xF3 "test registers"
// that ov7670.c uses are not in the device register map (RM datasheet Table 5
// ends at 0xC9), so that path is a Renode-only fiction and is not used here.
//
// De-interleaving Y from the chroma bytes is done on the CPU. STM32F4 has no
// usable byte-select: RM0090 Table 68 marks DCMI_CR bits 31:15 as "Reserved,
// must be kept at reset value", even though stm32f446xx.h defines
// DCMI_CR_BSM_0/1/OEBS/LSM/OELS at bits 16-20. That header/RM disagreement is
// unresolved, so software stripping is the default. Define
// NCAP_DCMI_TRY_BSM=1 to set those bits and see whether the silicon honours
// them; if it does, chroma never enters RAM and the QVGA budget changes.

// Capture geometry. VIDEO_WIDTH/VIDEO_HEIGHT come from the build (the same
// tunables the UDP stream uses) so the sensor output and the emitted frames
// agree.

// Frames buffered before the DMA wraps. 2 gives real producer/consumer slack;
// 1 is the minimum that can still work in-place.
#ifndef DVP_FRAMES
#define DVP_FRAMES 2
#endif

// Rows captured from the sensor starting at row 0. 0 = the sensor's full output
// height (VIDEO_HEIGHT). A smaller value enables DCMI crop
// (CR.CROP + CWSTRT/CWSIZE, RM0090 13.8.9/13.8.10) so a full-width window fits
// in RAM: 320x240 YUV422 is 153600 B/frame and cannot fit in 128 KB at all,
// while 320x160 is 102400 B.
#ifndef DVP_CROP_ROWS
#define DVP_CROP_ROWS 0
#endif

#if DVP_CROP_ROWS > 0
#define DVP_STREAM_H  ((uint16_t)DVP_CROP_ROWS)
#else
#define DVP_STREAM_H  ((uint16_t)VIDEO_HEIGHT)
#endif
//
// YUV420 is deliberately NOT offered. The OV7670 has no YUV420 output mode -
// datasheet COM7[2]/[0] selects YUV / RGB / Bayer RAW / processed Bayer RAW,
// and "YUV420" appears nowhere in the document - so a YUV420 build could never
// have matched real silicon.
//
// A crop window makes the DMA slot smaller than a sensor frame, so the DMA must
// be re-armed on every frame boundary and any drift corrupts the image. That
// works because capture is run in DCMI snapshot mode (CR.CM): capture stops at
// each frame end and the model's CR.CAPTURE is only honoured at the next frame
// start, so the DMA's first byte is always the first byte of the window. See
// dvp_publish_and_rearm() in ov7670_dvp.c.

// Bytes the DMA receives per frame. With DVP_LUMA_ONLY the DCMI drops the
// chroma bytes in hardware (byte select mode) so a frame is luma only, 1 byte
// per pixel; otherwise the whole YUYV line crosses the bus and chroma has to be
// stripped afterwards.
#ifndef DVP_LUMA_ONLY
#define DVP_LUMA_ONLY 0
#endif

#if DVP_LUMA_ONLY
#define DVP_FRAME_BYTES  ((uint32_t)VIDEO_WIDTH * DVP_STREAM_H)
#else
#define DVP_FRAME_BYTES  ((uint32_t)VIDEO_WIDTH * DVP_STREAM_H * 2u)
#endif

#define DVP_LUMA_FRAME_BYTES (VIDEO_WIDTH * VIDEO_HEIGHT)
#define DVP_RING_BYTES       ((uint32_t)DVP_FRAMES * DVP_FRAME_BYTES)

bool ov7670_dvp_init(void);

// True once at least one frame has been captured and de-interleaved.
bool ov7670_dvp_frame_ready(void);

// Hands the newest completed luma frame to the caller and re-arms the DMA.
// Returns false if no frame was ready. On return the buffer holds
// VIDEO_WIDTH*VIDEO_HEIGHT luma bytes with rows packed contiguously.
bool ov7670_dvp_take_frame(const uint8_t **luma);

// Frame counters for diagnostics.
uint32_t ov7670_dvp_frames_captured(void);
uint32_t ov7670_dvp_overruns(void);

// Frames accepted / rejected by the row-0 coherence check (see below).
uint32_t ov7670_dvp_frames_accepted(void);
uint32_t ov7670_dvp_frames_rejected(void);

// Row-0 coherence check. With a crop the DMA must be re-armed exactly on each
// frame boundary; if the re-arm slips, the capture starts part-way down the
// sensor frame and every row is offset. That shows up immediately in row 0,
// which then holds an arbitrary sensor line instead of the top one.
//
// The test compares row 0 of the candidate frame against row 0 of the last
// ACCEPTED frame. Consecutive frames of a real scene are temporally coherent in
// their first row, so a large mean absolute difference means the capture
// started in the wrong place. It makes no assumption about scene content, only
// about frame-to-frame stability, which is a property of real sensors.
//
// MEASURED AND NOT EFFECTIVE for the crop path, so it defaults to off: the
// re-arm slip is *consistent* from frame to frame, so a misaligned frame is as
// temporally coherent in row 0 as an aligned one and the check passes it. It can
// only detect a frame whose offset differs from its predecessor.
#ifndef DVP_ALIGN_CHECK
#define DVP_ALIGN_CHECK 1
#endif

// Reject when the mean absolute row-0 difference exceeds this many levels.
#ifndef DVP_ALIGN_TOLERANCE
#define DVP_ALIGN_TOLERANCE 48
#endif

#endif
