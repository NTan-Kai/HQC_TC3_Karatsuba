# Use this only when vect_mul_top / karatsuba_top is the synthesis top and its clock port is CLK.
create_clock -period 4.000 -name CLK_250M [get_ports CLK]
