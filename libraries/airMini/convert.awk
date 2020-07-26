BEGIN {
   found = 0;
   n = 0;
}

(!found) {
   if ($0 !~/IOCFG2/) {
      next;
   }
   else found = 1;
}

{
   gsub(/,/," ");
   val[n++] = $1;
   if ($0 ~ /RCCTRL0/) exit;
}

END {

   for(i=0;i<n;i++) {
      printf("%s,",val[i]);
   }

}
