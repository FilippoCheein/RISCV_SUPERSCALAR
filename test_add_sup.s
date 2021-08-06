	
addi a0, zero, 50 #50 is an arbitrary number, most tests will end up with a result of 50 to keep things simple
addi a1, zero, 1

nop
nop

addi a2, zero, 0
addi t0, zero, 25

add a2, a0, a1
sub t0, a0, a1

las 
laigen