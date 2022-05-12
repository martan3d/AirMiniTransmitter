BEGIN {
   ival=0;
}

/^ *#/ {next;}

{ 
   sub(/^.*{/,"");
   sub(/,.*/,"");
   sub(/}.*/,"");
   sub(/^ */,"");
   val[++ival] = $1;
}

END {

   printf("%s ",FILENAME);
   for(i=0;i<=ival;i++) {
      printf("%s ", val[i]);
   }
   printf("\n");

}
