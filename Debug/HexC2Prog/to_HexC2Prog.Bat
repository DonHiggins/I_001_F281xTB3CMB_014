Rem - - - - - - to_HexC2Prog.Bat - - - - - -
Rem
Rem Convert TI .out file to hex format
Rem
Rem - - - - - - - - - - - - - - - - - - - 
cd ..
copy TB3CM_DSP_Code.out HexC2Prog\* /Y
copy TB3CM_DSP_Code.map HexC2Prog\* /Y
cd HexC2Prog
C:\ti\ccsv5\tools\compiler\c2000_6.1.0\bin\hex2000 -romwidth 16 -memwidth 16 -i -O TB3CM_DSP_Code.hex TB3CM_DSP_Code.out
Rem
pause


