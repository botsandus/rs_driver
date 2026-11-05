# Safety Airy regression fixtures

These three UDP payloads were received from a pre-production 96-channel Safety
Airy on 2026-10-20. They are packet data, not firmware or executable code.

- `msop.bin`: 1240 bytes, CRC `d8dd3d2d` at byte 1230.
- `difop.bin`: 1248 bytes, CRC `473ed739` at byte 1242.
- `imu.bin`: 51 bytes; no established CRC.

Firmware IDs: overall `00023300`, top `11020411`, bottom `11020313`,
application `B0721162`, motor `260128A0`, CGI `F000030B`.

The supplied Safety Airy User Guide sections 4.8 and Appendix B establish the
basic layouts. Static inspection of SafeParamTool 3.6.0
`libhost.lidar.dll` (SHA-256
`039f2c5a6e672785de466a3e5c8623d7a44ffaa202546562c04c45c5312f7ddb`)
establishes the distance mask, optical offsets and factory firing schedule.
`libinfrastructure.dll` (SHA-256
`3bed5245949c761d7e7812b65905d9ff8d4238ea606797003d109004dd360606`)
establishes the CRC algorithm, separately validated against 35,838 MSOP and
1,200 DIFOP packets. The DLLs are not included.

`b1_rear_msop.bin` / `b1_rear_difop.bin` and `b1_front_msop.bin` /
`b1_front_difop.bin` were captured from two updated B1 samples on 2026-11-05.
Serials are `300bbfcd0061` and `300bbfcd0071`. Both report overall firmware
`00040400`, application `b1720003`, and MSOP type `0x0b` at byte 31. Their
96-channel profile, paired blocks, range encoding and microsecond timestamp
format match the existing decoder. A two-second capture checked 11,961 MSOP
and 398 DIFOP packets without CRC, framing or paired-block errors. Geometry
test values are independently calculated from each unit's captured ranges
and DIFOP calibration; they are not a physical calibration certification.
