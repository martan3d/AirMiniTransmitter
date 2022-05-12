#! /opt/local/bin/gawk -f
# Created by: Darrell R. Lamm, darrelllamm0@gmail.com
# Purpose: Compute the band width and baud rate for the CC1101 using FSCTRL1 and FSCTRL0
# Note: You must use gawk, not the garden-variety version of awk
BEGIN {
   print "Enter FSCTRL1 FSCTRL0 fXOSC:"
}

{
   gsub(/,/," ");
}

NF < 3 {print; next;}

{
   FSCTRL1 = strtonum($1); # for conversion of hex
   FSCTRL0 = strtonum($2); # for conversion of hex
   fXOSC = $3;
   $1 = $2 = $3 = "";
   print "Comment: " $0;

   FREQ_IF =  and(FSCTRL1,0x0F);
   FREQOFF = and(FSCTRL0,0xFF);
   FSCTRL1h = sprintf("0x%X",FSCTRL1);
   FSCTRL0h = sprintf("0x%X",FSCTRL0);

   print "FSCTRL1: " FSCTRL1h " FSCTRL0: " FSCTRL0h " fXOSC: " fXOSC;
   FREQ_IFh = sprintf("0x%X",FREQ_IF);
   FREQOFFh = sprintf("0x%X",FREQOFF);
   print "FREQ_IF: " FREQ_IFh " fXOSC: " fXOSC " IF Frequency: " fIF(FREQ_IF, fXOSC) " Hz";
   print "FREQOFF: " FREQOFFh " fXOSC: " fXOSC " Frequency Offset: " fOFFSET(FREQOFF, fXOSC) " Hz";
   print "---------------";
}

{
   print "Enter FSCTRL1 FSCTRL0 fXOSC:"
}


function fIF(FREQ_IF, fXOSC) {
   return fXOSC*FREQ_IF/2**10;
}

function fOFFSET(FREQOFF, fXOSC) {
   return twos_complement(FREQOFF,8);
}

function twos_complement(input_value, num_bits) {
   mask = 2**(num_bits-1);
   mask_complement = compl(mask);
   return -and(input_value,mask) + and(input_value,mask_complement);
}
