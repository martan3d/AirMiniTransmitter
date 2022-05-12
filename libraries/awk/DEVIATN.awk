#! /opt/local/bin/gawk -f
# Created by: Darrell R. Lamm, darrelllamm0@gmail.com
# Purpose: Compute fDev from DEVIATN and fXOSC
# Note: You must use gawk, not the garden-variety version of awk
BEGIN {
   entry_line = "Enter DEVIATN fXOSC:"
   print entry_line;
}

{
   gsub(/,/," ");
}

NF < 2 {print; next;}

{
   DEVIATN = strtonum($1); # for conversion of hexstr
   fXOSC = $2;
   $1 = $2 = "";
   print "Comment " $0;

   DEVIATNh = hexstr(DEVIATN);

   print "DEVIATN: " DEVIATNh " fXOSC: " fXOSC;;

   DEVIATION_E = convert(DEVIATN,6,4);
   DEVIATION_M = convert(DEVIATN,2,0);

   DEVIATION_Eh = hexstr(DEVIATION_E);
   DEVIATION_Mh = hexstr(DEVIATION_M);
   print "DEVIATION_E: " DEVIATION_Eh " DEVIATION_M: " DEVIATION_Mh " fXOSC: " fXOSC " fDev: " fDev(DEVIATION_E, DEVIATION_M, fXOSC) " Hz";
   print "---------------";
   
}

{
   print entry_line;
}

function fDev(DEVIATION_E,DEVIATION_M,fXOSC) {
   return fXOSC*(8+DEVIATION_M)*2**DEVIATION_E/2**17;
}

function convert(val, high, low) {
   return and(2**(high-low+1)-1, rshift(val,low));
}

function hexstr(val) {
   return sprintf("0x%X",val);
}
