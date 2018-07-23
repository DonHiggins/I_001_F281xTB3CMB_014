TB3CM_DSP_Code.out
--outfile=TB3CM_DSP_Code.Mcs
--map=MapOut.map
--intel
--romwidth=16
--image

SECTIONS
{
  bootentry
  .text
  .cinit
  .econst
  ramfuncs
  .switch
}

ROMS
{
   ROM1: org=0x0003D8000, len=0x08000, romwidth 16, fill=0xFFFF
}