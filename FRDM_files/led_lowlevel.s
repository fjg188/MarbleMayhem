

.section .text

.global led_off
.type led_off %function

.global loop_3cycles
.type loop_3cycles %function

//Binary code for set_led
.section .text
.global set_led
.type set_led %function

	.equ GPIOE_BASEADDR, 0x400FF100
	.equ BITMASK_PINTHREE,  0x08
	.equ PSOR_OFFSET, 0x04
	.equ PCOR_OFFSET, 0x08


//Small timing loop to create predictable delays
.balign 4
loop_3cycles:
	SUB R0,#1
	BNE loop_3cycles
	BX LR

.balign 4
led_off:
	PUSH {LR}
	MOV R0, #0
	BL set_led
	POP {PC}


set_led:
	PUSH {R4, R5, LR}
	MOV R1, #24
	LDR R4, =GPIOE_BASEADDR
	MOV R5, #BITMASK_PINTHREE
	LSL R0, R0, #8

get_bits:
	CMP R1, #0
	BEQ done
	SUB R1, R1, #1 //Subtract from the counter
	LSL R0, R0, #1 //Shift bits left (any bit that moves past MSB is stored in carry flag
	BCS code_one_pulse  //If the Carry Flag is set, branch to code_one_pulse
	B code_zero_pulse   //If the carry flag was not set, branch to code_zero_pulse

code_one_pulse:
	STR R5, [R4, #PSOR_OFFSET]
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP          //T1H: 3 cycles STR + 30 cycles NOP = 33*20.9ns => 689.7 ns (within 580-1000 ns range)
	STR R5, [R4, #PCOR_OFFSET]
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP          //T1L: 3 cycles STR + 30 cycles NOP = 33*20.9ns => 689.7 ns (within 580-1000 ns range)
	B get_bits

code_zero_pulse:
	STR R5, [R4, #PSOR_OFFSET]
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP			//T0H: 3 cycles STR + 10 cycles NOP = 13*20.9ns => 271.7 ns (within 220-380 ns range)
	STR R5, [R4, #PCOR_OFFSET]
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP          //T0L: 3 cycles STR + 30 cycles NOP = 33*20.9ns => 689.7 ns (within 580-1000 ns range)
	B get_bits

done:
	POP {R4, R5, PC}
	BX LR
	.end
