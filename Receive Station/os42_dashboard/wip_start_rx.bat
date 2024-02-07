@ECHO OFF

echo OS-42 LIVE IMAGERY RX
echo WRITTEN BY SASHA VE3SVF

echo Start RTL-SDR and FSK Demodulator...

rtl_sdr -f 433600000 -s 2400000 -g 42 - | fsk_demod --cu8 --stats=100 2 2400000 300000 - - > test.egg 2> python fskdemodgui.py