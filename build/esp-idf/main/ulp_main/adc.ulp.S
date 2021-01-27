       
       
 .set adc_channel, 6
 .set adc_oversampling_factor_log, 2
 .set adc_oversampling_factor, (1 << adc_oversampling_factor_log)
 .bss
 .global Tpig_detect
Tpig_detect:
 .long 0
 .global last_result
last_result:
 .long 0
 .global io_number
io_number:
 .long 0
 .text
 .global entry
entry:
 move r0, 0
 stage_rst
measure:
 adc r1, 0, adc_channel + 1
 add r0, r0, r1
 stage_inc 1
 jumps measure, adc_oversampling_factor, lt
 rsh r0, r0, adc_oversampling_factor_log
 move r3, last_result
 st r0, r3, 0
 move r3, Tpig_detect
 ld r3, r3, 0
 sub r3, r3, r0
 jump wake_up, ov
 move r3, io_number
 ld r3, r3, 0
 move r0, r3
 jumpr read_io_high, 16, ge
 REG_RD ((((0x3ff48400 + 0x24)) - 0x3ff48000) / 4), ((14) + (16) - 1), (14)
 rsh r0, r0, r3
 and r0, r0, 1
 jumpr wake_up , 1 , EQ
read_io_high:
 REG_RD ((((0x3ff48400 + 0x24)) - 0x3ff48000) / 4), ((14 + 16) + (2) - 1), (14 + 16)
 sub r3, r3, 16
 rsh r0, r0, r3
 .global exit
exit:
 halt
 .global wake_up
wake_up:
 REG_RD ((((0x3ff48000 + 0xc0)) - 0x3ff48000) / 4), ((19) + ((((((((0x1) >> (0)) & 1) + (((0x1) >> (0 + 1)) & 1)) + ((((0x1) >> (0 + 2)) & 1) + (((0x1) >> (0 + 2 + 1)) & 1))) + (((((0x1) >> (0 + 4)) & 1) + (((0x1) >> (0 + 4 + 1)) & 1)) + ((((0x1) >> (0 + 4 + 2)) & 1) + (((0x1) >> (0 + 4 + 2 + 1)) & 1)))) + ((((((0x1) >> (0 + 8)) & 1) + (((0x1) >> (0 + 8 + 1)) & 1)) + ((((0x1) >> (0 + 8 + 2)) & 1) + (((0x1) >> (0 + 8 + 2 + 1)) & 1))) + (((((0x1) >> (0 + 8 + 4)) & 1) + (((0x1) >> (0 + 8 + 4 + 1)) & 1)) + ((((0x1) >> (0 + 8 + 4 + 2)) & 1) + (((0x1) >> (0 + 8 + 4 + 2 + 1)) & 1)))))) - 1), (19)
 and r0, r0, 1
 jump exit, eq
 wake
 REG_WR ((((0x3ff48000 + 0x18)) - 0x3ff48000) / 4), ((24) + (((((((0x1) >> (0)) & 1) + (((0x1) >> (0 + 1)) & 1)) + ((((0x1) >> (0 + 2)) & 1) + (((0x1) >> (0 + 2 + 1)) & 1))) + (((((0x1) >> (0 + 4)) & 1) + (((0x1) >> (0 + 4 + 1)) & 1)) + ((((0x1) >> (0 + 4 + 2)) & 1) + (((0x1) >> (0 + 4 + 2 + 1)) & 1))))) - 1), (24), ((((0) & 0x1)) & 0xff)
 halt
