	.syntax unified
	.eabi_attribute 6, 2
	.eabi_attribute 8, 1
	.eabi_attribute 9, 1
	.eabi_attribute 20, 1
	.eabi_attribute 21, 1
	.eabi_attribute 23, 3
	.eabi_attribute 24, 1
	.eabi_attribute 25, 1
	.file	"/home/xlous/Development/compiler/llvm-idem/llvm/test/CodeGen/ARM/fpowi.ll"
	.text
	.globl	_ZSt3powdi
	.align	2
	.type	_ZSt3powdi,%function
_ZSt3powdi:                             @ @_ZSt3powdi
@ BB#0:                                 @ %entry
	push	{r11, lr}
	orreq	r0, r0, r0
	bl	__powidf2
	orreq	r0, r0, r0
	pop	{r11, lr}
	mov	pc, lr
.Ltmp0:
	.size	_ZSt3powdi, .Ltmp0-_ZSt3powdi


