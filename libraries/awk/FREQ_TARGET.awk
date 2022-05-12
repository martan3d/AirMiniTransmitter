#! /opt/local/bin/gawk -f
# Created by: Darrell R. Lamm, darrelllamm0@gmail.com
# Purpose: Compute frequency from FREQ[210]
# Note: You must use gawk, not the garden-variety version of awk
BEGIN {
   print "Enter Target and fXOSC frequency"
}

{
   gsub(/,/," ");
}

{
   FREQ = $1+0.0;
   fXOSC = $2+0.0;
   $1 = $2 = "";
   print "Comment: " $0;

   FREQI = int(FREQ*2**16/fXOSC);
   FREQH = sprintf("0x%X", FREQI);
   print "FREQ: " FREQ " fXOSC: " fXOSC " (FREQ - FREQOUT): " FREQ - FREQI*fXOSC /2**16 " FREQH: "  FREQH ;
   FREQ2 = rshift(FREQI,16);
   FREQ2h = sprintf("0x%X",FREQ2);
   FREQ1 = rshift(and(FREQI,0x00FFFF),8); 
   FREQ1h = sprintf("0x%X",FREQ1);
   FREQ0 = rshift(and(FREQI,0x0000FF),0); 
   FREQ0h = sprintf("0x%X",FREQ0);
   print "FREQ2: " FREQ2h " FREQ1: " FREQ1h " FREQ0: " FREQ0h;
}

