BEGIN {
   found = 0;
   n = 0;
   val[n++] = "0x40";
}

(!found) {
   if ($0 !~/IOCFG2/) {
      next;
   }
   else found = 1;
}

{
   if ($0 ~ /};/) exit;
   gsub(/,/," ");
   val[n++] = $1;
}

END {

   for(i=0;i<n;i++) {
      printf("%s,",val[i]);
   }

}
