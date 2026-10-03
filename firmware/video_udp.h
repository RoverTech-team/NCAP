// Grayscale (luma-only) video feed as a continuous UDP stream.
//
// Chosen over TCP because the W5500 model's UDP path is already proven
// byte-exact (it carries the entire micro-ROS link), whereas the TCP path
// depends on ioLibrary's Sn_TX_W streaming semantics that the model does not
// reproduce exactly. For live video this is also the better transport: no
// retransmits, and losing a late frame is the correct behaviour.
//
// The W5500 terminates UDP/IP in hardware, so there is no UDP stack here: one
// socket, sendto() per datagram. Frames are generated and sent one row block at
// a time, so peak RAM is a single row block - there is no full-frame buffer.
//
// Wire format, one datagram per row block, self-describing so no state is
// needed on the receiver side:
//   'V' 'F' 'R' '0' | u32 seq | u16 width | u16 height
//                  | u16 row_offset | u16 rows | u32 payload_len | luma...
// A stream starts with a meta datagram (magic 'V''F''M''0') whose payload is a
// human-readable HTTP-style preamble.

#ifndef VIDEO_UDP_H
#define VIDEO_UDP_H

#include <stdint.h>
#include <stdbool.h>

// Streams synthetic grayscale luma frames to VIDEO_HOST_IP:VIDEO_PORT.
// Never returns.
void video_udp_serve(void);

// Stepwise form for running alongside micro-ROS: open the socket and send the
// preamble once, then call video_udp_step() to emit a single row block. It
// returns true when a frame completes. Uses VIDEO_SOCK, which must differ from
// the micro-ROS transport socket.
void video_udp_init(void);
bool video_udp_step(void);

#endif