BEGIN {
   N=3;
}

{
   N = $1;
   exit;
}

END {
   permute(0,N,permutation);
}

function permute(depth,maxdepth,permutation,       i,j,k,distance,ilast) {
   #print "depth, maxdepth: ", depth, maxdepth;
   if (depth == maxdepth) {
      printf("( ");
      distance = 0;
      distance2 = 0;
      k=0;
      for(i in permutation) {
         printf("%d ",permutation[i]);
         if (k>0) {
            di = abs(permutation[i]-permutation[ilast]);
            distance += di
            distance2 += di*di
         }
         k++;
         ilast = i;
      }
      mom1 = distance/(maxdepth-1);
      mom2 = distance2/(maxdepth-1);
      rms = mom2-mom1*mom1
      print "): distance: " distance " avg: " mom1 " rms: " rms;
      return;
   }
   for(i=0;i<maxdepth;i++) {
      found = 0;
      for (j=0;j<depth;j++) {
         if (permutation[j] == i) {
            found = 1;
            break;
         }
      }
      if (!found) {
         permutation[depth]=i;
         permute(depth+1,maxdepth,permutation);
      }
   }
   return;
}

function abs(x) {
   return x > 0 ? x : -x;
}
