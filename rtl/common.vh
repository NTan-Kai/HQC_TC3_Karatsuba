`define PARAM_N 17669 //hqc-128
//`define PARAM_N 35851 //hqc-192
//`define PARAM_N 57637 //hqc-256 
`define CEIL_DIVIDE(a, b) (((a)/(b)) + (((a) % (b)) == 0 ? 0 : 1))
`define VEC_N_SIZE_64 `CEIL_DIVIDE(`PARAM_N, 64)
`define DATA_WIDTH 64
`define SAFE_CLOG2(x) (((x) <= 1) ? 1 : $clog2(x))

`define ADDR_WIDTH      18
`define MODE_ADDR_WIDTH 2

`define AXI_DONE_ADDR           40'h04_0004_0000
`define AXI_START_ADDR          40'h04_0008_0000
`define AXI_TRANSFER_MASK       16'h04_0

`define AXI_DATA_WIDTH          128