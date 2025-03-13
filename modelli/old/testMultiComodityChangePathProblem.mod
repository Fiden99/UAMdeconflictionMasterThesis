#data
param nf;																		#number of flights
param nn;																		#number of nodes
set F:= 0..nf-1;																#set of flights
set V:= 0..nn-1;																#set of nodes
set E within {V cross V};														#arcs
param s{F};																		#starting point for each flight
param e{F};																		#ending poitn for each flight
param d{E}>=0 default 0;											            #distance for each pair of nodes
param v_max{F,E};
param v_min{F,E};
param dMCF{F,E} >=0 default 0;
param bigM:= 1000;	                                                            #bigM for linearizing purpose
param angleM{x in V, (x1,x) in E, (x2,x) in E: x1<>x2};							# angle-for merging
param angleP{x in V, (x,x1) in E, (x,x2) in E: x1<>x2};							# angle+ for splitting
param anglePM{x in V,(x,x1) in E, (x2,x) in E/*: x1<>x2*/};							#angle -+ divering
param D;																		# safety distance
param t_hat_ear{F,V} default 0;
param t_hat_lat{F,V} default 0;
param w{(i,j) in E,F} binary;													#flight f pass through arc i,j
#param and set built
set FV = {(f,v) in (F cross V) : 
             exists {(i,j) in E: (j == v or i == v)} w[i,j,f] == 1};
#set UniqueValues = setof{f in F, v in V} t_hat_ear[f,v];
#param gapTime default 10;
#param costConf default 10000;
#variables
var t_down  {(i,x) in FV: x<>s[i]} integer>= 0;
var t_up    {(i,x) in FV: x<>s[i]} integer >= 0;
var t_ear   {FV} integer >=0 ;														    #variable time
var t_lat   {FV}  integer >=0 ;                                                            #variable time                                  
var t_ear_start{F} >=0 integer;			                                        #variable to make the first flight starting at integer time													
var l{i in F, j in F, x in V: (i,x) in FV and (j,x) in FV and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))} binary;

#heuristic var
var wPath{(i,j) in E, F} binary;	
#var conflict{i in F,j in F,V : i < j and t_hat_ear[i,s[i]] <= t_hat_ear[j,s[j]] and t_hat_ear[j,s[j]]- t_hat_ear[i,s[i]] <= gapTime } binary;	

#test with sets
set differentAngle=(setof{x in V, (x1,x) in E, (x2,x) in E: x1<>x2} angleM[x,x1,x2]) union  (setof{x in V, (x,x1) in E, (x,x2) in E: x1<>x2} angleP[x,x1,x2]) union (setof{x in V,(x,x1) in E, (x2,x) in E} anglePM[x,x1,x2]);

set allConf={i in F, j in F, x in V, differentAngle:i<>j and  (i,x) in FV and (j,x) in FV and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))};

set diver1Conf={(i,j,x,dA) in allConf,(x,x1) in E, (x2,x) in E: i<>j and w[x,x1,i] + w[x2,x,j]== 2 and anglePM[x,x1,x2]==dA};
set diver2Conf={(i,j,x,dA) in allConf,(x,x1) in E, (x2,x) in E: i<>j and w[x,x1,j] + w[x2,x,i]== 2  and anglePM[x,x1,x2]==dA};
set diver1SetOf=setof{(i,j,x,dA,x1,x2) in diver1Conf} (i,j,x,dA);
set diver2SetOf=setof{(i,j,x,dA,x1,x2) in diver2Conf} (i,j,x,dA);

set trailConf={(i,j,x,dA) in allConf, (x,y) in E: i<>j and w[x,y,i]+w[x,y,j]==2};
set trail1SetOf=setof{(i,j,x,dA,y) in trailConf} (i,j,x,dA);
set trail2SetOf=setof{(i,j,x,dA,y) in trailConf} (i,j,y,dA);

set splitConf={(i,j,x,dA) in allConf,(x,x1) in E, (x,x2) in E: i<>j and x1<>x2 and w[x,x1,i]+w[x,x2,j]==2 and angleP[x,x1,x2]==dA};
set splitSetOf=setof{(i,j,x,dA,x1,x2) in splitConf} (i,j,x,dA);

set mergeConf={(i,j,x,dA) in allConf,(x1,x) in E, (x2,x) in E: x1<>x2 and i<>j and w[x1,x,i]+w[x2,x,j]==2 and angleM[x,x1,x2]==dA};

set mergeSetOf=setof{(i,j,x,dA,x1,x2) in mergeConf} (i,j,x,dA);


set diver1 = setof{(i,j,x,dA,x1,x2) in diver1Conf} (i,j,x,x1,x2);
set diver2 = setof{(i,j,x,dA,x1,x2) in diver2Conf diff diver1Conf} (i,j,x,x1,x2);
set merge  = setof{(i,j,x,dA,x1,x2) in mergeConf: (i,j,x,dA) not in (diver1SetOf union diver2SetOf)} (i,j,x,x1,x2);
set split  = setof{(i,j,x,dA,x1,x2) in splitConf: (i,j,x,dA) not in (diver1SetOf union diver2SetOf union mergeSetOf)} (i,j,x,x1,x2);
set trail1 = setof{(i,j,x,dA,y) in trailConf:(i,j,x,dA) not in (diver1SetOf union diver2SetOf union mergeSetOf union splitSetOf)} (i,j,x,y);
set trail2 = setof{(i,j,x,dA,y) in trailConf:(i,j,y,dA) not in (diver1SetOf union diver2SetOf union mergeSetOf union splitSetOf)} (i,j,x,y);


/*
var deviation{F,V};
subject to defDev{i in F, x in V}:
dev[i,x] >= t_ear[i,x] - t_hat_ear[i,x];
subject to defDev2{i in F, x in V}:
dev[i,x] >= t_hat_ear[i,x] - t_ear[i,x];
*/


subject to startingW{i in F}:
sum{x in V: (s[i],x) in E} w[s[i],x,i]=1;
subject to finishingW{i in F}:
sum{x in V: (x,e[i]) in E} w[x,e[i],i]=1;
subject to allW{i in F, x in V :  x <> s[i] and x <> e[i]}:
sum{y in V: (x,y) in E} w[x,y,i]=sum{y in V: (y,x) in E} w[y,x,i];
/*
subject to definenumConf1{(x,y) in E, i in F, j in F: i<>j}:
numConflicts[x,y,i,j] <=  wConf[x,y,i];
subject to definenumConf2{(x,y) in E, i in F, j in F: i<>j}:
numConflicts[x,y,i,j] <=  wConf[x,y,j];
subject to definenumConf3{(x,y) in E, i in F, j in F: i<>j}:
numConflicts[x,y,i,j] >=  wConf[x,y,i] + wConf[x,y,j] - 1;

subject to startingPathConf{i in F}:
sum{(x,s[i]) in E} wConf[x,s[i],i] -sum{(s[i],x) in E} wConf[s[i],x,i]=-1;
subject to finishingPathConf{i in F}:
sum{(x,e[i]) in E} wConf[x,e[i],i]- sum{(e[i],x) in E} wConf[e[i],x,i]=1;
subject to allPathConf{i in F, x in V :  x <> s[i] and x <> e[i]}:
sum{(x,y) in E} wConf[x,y,i]=sum{(y,x) in E} wConf[y,x,i];
*/

subject to startingPath{i in F}:
sum{(x,s[i]) in E} wPath[x,s[i],i] - sum{(s[i],x) in E} wPath[s[i],x,i] = -1;
subject to finishingPath{i in F}:
sum{(x,e[i]) in E} wPath[x,e[i],i] - sum{(e[i],x) in E} wPath[e[i],x,i] = 1;
subject to allPath{i in F, x in V :  x <> s[i] and x <> e[i]}:
sum{(x,y) in E} wPath[x,y,i]=sum{(y,x) in E} wPath[y,x,i];


subject to afterprecalculated{f in F}:
t_hat_ear[f,s[f]] <= t_ear_start[f];
subject to calculateLat{(i,x) in FV}:
t_lat[i,x]=t_ear[i,x] + t_hat_lat[i,x] - t_hat_ear[i,x];

subject to limitT_down{(i,x) in FV : x <> s[i]}:
t_down[i,x] <= t_ear[i,x];
subject to limitT_up{(i,x) in FV: x <> s[i]}:
t_ear[i,x] <= t_up[i,x];

subject to defineT_down{(f,y) in FV, (x,y) in E: y <> s[f] and w[x,y,f] == 1}:	
t_down[f,y] = t_ear[f,x] + d[x,y]/v_max[f,x,y];
subject to defineT_up{(f,y) in FV,(x,y) in E : y <> s[f] and w[x,y,f]==1}: 		
t_up[f,y]=t_ear[f,x] + d[x,y]/v_min[f,x,y];

subject to startInt{f in F}:
t_ear_start[f] = t_ear[f,s[f]];
#vedere come procedere: in mercedes viene detto che hat_t_ear[i,m] < (considerare anche =) t_hat_ear[j,m]
#è possibile che il risultato ottenuto sia confrontabile con quello che ho ottenuto indicando che i < j

/*

subject to trail13 {i in F, j in F,(x,y) in E: i<>j and w[x,y,i]+w[x,y,j]==2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[j,x]-t_lat[i,x] >= D/v_min[i,x,y] * l[i,j,x]- bigM*(1-l[i,j,x]);
subject to trail14 {i in F, j in F,(x,y) in E: i<>j and w[x,y,i]+w[x,y,j]==2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[i,x]-t_lat[j,x] >= D/v_min[j,x,y] * (1-l[i,j,x]) - bigM*l[i,j,x];

subject to trail23 {i in F, j in F,(x,y) in E: i<>j and w[x,y,i]+w[x,y,j]==2 and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
t_ear[j,y]-t_lat[i,y]>= D/v_min[j,x,y]* l[i,j,x] - bigM*(1-l[i,j,x]);
subject to trail24 {i in F, j in F,(x,y) in E: i<>j and w[x,y,i]+w[x,y,j]==2 and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
t_ear[i,y]-t_lat[j,y]>= D/v_min[i,x,y] * (1-l[i,j,x]) - bigM*l[i,j,x];

*/
#subject to trail13 {i in F, j in F,(x,y) in E: i<>j and w[x,y,i]+w[x,y,j]==2 and (x == s[i] or x == s[j] or y == e[i] or y== e[j]) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
subject to trail13 {(i,j,x,y) in trail1}:
t_ear[j,x]-t_lat[i,x] >= D/v_min[i,x,y] * l[i,j,x]- bigM*(1-l[i,j,x]);
#subject to trail14 {i in F, j in F,(x,y) in E: i<>j and w[x,y,i]+w[x,y,j]==2 and (x == s[i] or x == s[j] or y == e[i] or y== e[j]) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
subject to trail14 {(i,j,x,y) in trail1}:
t_ear[i,x]-t_lat[j,x] >= D/v_min[j,x,y] * (1-l[i,j,x]) - bigM*l[i,j,x];

#subject to trail23 {i in F, j in F,(x,y) in E: i<>j and w[x,y,i]+w[x,y,j]==2 and (x == s[i] or x == s[j] or y == e[i] or y== e[j]) and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
subject to trail23 {(i,j,x,y) in trail2}:
t_ear[j,y]-t_lat[i,y]>= D/v_min[j,x,y]* l[i,j,y] - bigM*(1-l[i,j,y]);
#subject to trail24 {i in F, j in F,(x,y) in E: i<>j and w[x,y,i]+w[x,y,j]==2 and (x == s[i] or x == s[j] or y == e[i] or y== e[j]) and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
subject to trail24 {(i,j,x,y) in trail2}:
t_ear[i,y]-t_lat[j,y]>= D/v_min[i,x,y] * (1-l[i,j,y]) - bigM*l[i,j,y];


#TODO check if the speed parameter is correct for merge4
#subject to merge3 {i in F, j in F,x in V,(x1,x) in E, (x2,x) in E: i<>j and x1<>x2 and (w[x1,x,i]+w[x2,x,j]==2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: 
subject to merge3{(i,j,x,x1,x2) in merge}:
t_ear[j,x]- t_lat[i,x]>=angleM[x,x1,x2]*D/v_min[j,x2,x] * l[i,j,x] - bigM*(1-l[i,j,x]);
#subject to merge3_1 {i in F, j in F,x in V,(x1,x) in E, (x2,x) in E: i<>j and x1<>x2 and (w[x1,x,j]+w[x2,x,i]==2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: 
#t_ear[j,x]- t_lat[i,x]>=angleM[x,x1,x2]*D/v_min[i,x2,x] * l[i,j,x] - bigM*(1-l[i,j,x]);

#subject to merge4 {i in F, j in F,x in V,(x1,x) in E, (x2,x) in E: i<>j and x1<>x2 and (w[x1,x,i]+w[x2,x,j]==2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: 
subject to merge4{(i,j,x,x1,x2) in merge}:
t_ear[i,x]- t_lat[j,x]>=angleM[x,x1,x2]*D/v_min[i,x1,x]* (1-l[i,j,x]) - bigM*l[i,j,x];
#subject to merge4_1 {i in F, j in F,x in V,(x1,x) in E, (x2,x) in E: i<>j and x1<>x2 and (w[x1,x,j]+w[x2,x,i]==2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: 
#t_ear[i,x]- t_lat[j,x]>=angleM[x,x1,x2]*D/v_min[j,x1,x]* (1-l[i,j,x]) - bigM*l[i,j,x];

#subject to diver3 {i in F, j in F,x in V,(x,x1) in E, (x2,x) in E: i<>j /*and x1<>x2*/ and (w[x,x1,i] + w[x2,x,j]== 2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
subject to diver3 {(i,j,x,x1,x2) in diver1}:
t_ear[j,x]- t_lat[i,x]>=anglePM[x,x1,x2]*(D/v_min[j,x2,x]+D/v_min[i,x,x1])*l[i,j,x] - bigM*(1-l[i,j,x]);
#subject to diver3_1 {i in F, j in F,x in V,(x,x1) in E, (x2,x) in E: i<>j /*and x1<>x2*/ and (w[x,x1,j]+w[x2,x,i]== 2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
subject to diver3_1 {(i,j,x,x1,x2) in diver2}:
t_ear[j,x]- t_lat[i,x]>=anglePM[x,x1,x2]*(D/v_min[i,x2,x]+D/v_min[j,x,x1])* l[i,j,x] - bigM*(1-l[i,j,x]);

#subject to diver4 {i in F, j in F,x in V,(x,x1) in E, (x2,x) in E: i<>j /*and x1<>x2*/ and (w[x,x1,i] + w[x2,x,j]== 2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
subject to diver4 {(i,j,x,x1,x2) in diver1}:
t_ear[i,x]- t_lat[j,x]>=anglePM[x,x1,x2]*(D/v_min[j,x2,x]+D/v_min[i,x,x1])* (1-l[i,j,x]) - bigM*l[i,j,x];
#subject to diver4_1 {i in F, j in F,x in V,(x,x1) in E, (x2,x) in E: i<>j /*and x1<>x2*/ and (w[x,x1,j]+w[x2,x,i]== 2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
subject to diver4_1 {(i,j,x,x1,x2) in diver2}:
t_ear[i,x]- t_lat[j,x]>=anglePM[x,x1,x2]*(D/v_min[i,x2,x]+D/v_min[j,x,x1])* (1-l[i,j,x]) - bigM*l[i,j,x];

/*
subject to diver3SameNode {i in F, j in F,x in V,(x,x1) in E, (x2,x) in E: i<>j and x1==x2 and (w[x,x1,i] + w[x2,x,j]== 2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
t_ear[j,x]- t_lat[i,x]>=(D/v_min[j,x2,x]+D/v_min[i,x,x1])*l[i,j,x] - bigM*(1-l[i,j,x]);

subject to diver3_1SameNode {i in F, j in F,x in V,(x,x1) in E, (x2,x) in E: i<>j and x1==x2 and (w[x,x1,j]+w[x2,x,i]== 2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
t_ear[j,x]- t_lat[i,x]>=(D/v_min[j,x2,x]+D/v_min[i,x,x1])* l[i,j,x] - bigM*(1-l[i,j,x]);

subject to diver4SameNode {i in F, j in F,x in V,(x,x1) in E, (x2,x) in E: i<>j and x1==x2 and (w[x,x1,i] + w[x2,x,j]== 2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
t_ear[i,x]- t_lat[j,x]>=(D/v_min[i,x2,x]+D/v_min[j,x,x1])* (1-l[i,j,x]) - bigM*l[i,j,x];

subject to diver4_1SameNode {i in F, j in F,x in V,(x,x1) in E, (x2,x) in E: i<>j and x1==x2 and (w[x,x1,j]+w[x2,x,i]== 2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
t_ear[i,x]- t_lat[j,x]>=(D/v_min[i,x2,x]+D/v_min[j,x,x1])* (1-l[i,j,x]) - bigM*l[i,j,x];

*/

#change xy with explicit values that I have
#TODO check if the speed parameter is correct for split4

#subject to split3 {i in F, j in F,x in V,(x,x1) in E, (x,x2) in E: i<>j and x1<>x2 and (w[x,x1,i]+w[x,x2,j]==2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
subject to split3 {(i,j,x,x1,x2) in split}:
t_ear[j,x]- t_lat[i,x]>=angleP[x,x1,x2]*D/v_min[i,x,x1]*l[i,j,x] - bigM*(1-l[i,j,x]);
#subject to split3_1 {i in F, j in F,x in V,(x,x1) in E, (x,x2) in E: i<>j and x1<>x2 and (w[x,x1,j]+w[x,x2,i]==2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
#t_ear[j,x]- t_lat[i,x]>=angleP[x,x1,x2]*D/v_min[j,x,x1]*l[i,j,x] - bigM*(1-l[i,j,x]);
#subject to split4 {i in F, j in F,x in V,(x,x1) in E, (x,x2) in E: i<>j and x1<>x2 and (w[x,x1,i]+w[x,x2,j]==2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
subject to split4{(i,j,x,x1,x2) in split}: 
t_ear[i,x]- t_lat[j,x]>=angleP[x,x1,x2]*D/v_min[j,x,x2] * (1-l[i,j,x]) - bigM*l[i,j,x];
#subject to split4_1 {i in F, j in F,x in V,(x,x1) in E, (x,x2) in E: i<>j and x1<>x2 and (w[x,x1,j]+w[x,x2,i]==2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
#t_ear[i,x]- t_lat[j,x]>=angleP[x,x1,x2]*D/v_min[i,x,x2] * (1-l[i,j,x]) - bigM*l[i,j,x];


/*
subject to mergeConf1{i in F, j in F, x in V:i < j and t_hat_ear[i,s[i]] <= t_hat_ear[j,s[j]] and t_hat_ear[j,s[j]]- t_hat_ear[i,s[i]] <= gapTime}:
    conflict[i,j,x] >= sum{(x1,x) in E}wPath[x1,x,i] + sum{ (x2,x) in E}wPath[x2,x,j] - 1;

subject to diverConf1{i in F, j in F, x in V:i < j and t_hat_ear[i,s[i]] <= t_hat_ear[j,s[j]] and t_hat_ear[j,s[j]]- t_hat_ear[i,s[i]] <= gapTime}:
    conflict[i,j,x] >= sum{(x,x1) in E} wPath[x,x1,i] + sum{(x2,x) in E} wPath[x2,x,j] - 1;
subject to diverConf2{i in F, j in F, x in V:i < j and t_hat_ear[i,s[i]] <= t_hat_ear[j,s[j]] and t_hat_ear[j,s[j]]- t_hat_ear[i,s[i]] <= gapTime}:
    conflict[i,j,x] >= sum{(x,x1) in E} wPath[x,x1,j] + sum{(x2,x) in E} wPath[x2,x,i] - 1;

subject to splitConf1{i in F, j in F, x in V:i < j and t_hat_ear[i,s[i]] <= t_hat_ear[j,s[j]] and t_hat_ear[j,s[j]]- t_hat_ear[i,s[i]] <= gapTime}:
    conflict[i,j,x] >= sum{(x,x1) in E} wPath[x,x1,i] + sum{(x,x2) in E} wPath[x,x2,j] - 1;
*/


minimize UAM: sum{i in F} t_ear[i,e[i]];
minimize MC: sum{f in F, (x,y) in E} wPath[x,y,f] * dMCF[f,x,y];

#minimize MC: sum{f in F, (x,y) in E} wPath[x,y,f] * dMCF[x,y] + 
#    sum{i in F, j in F, x in V:i < j and t_hat_ear[i,s[i]] <= t_hat_ear[j,s[j]] and t_hat_ear[j,s[j]]- t_hat_ear[i,s[i]] <= gapTime}  
#    conflict[i,j,x]* costConf * (1+gapTime - (t_hat_ear[j,s[j]]-t_hat_ear[i,s[i]]));

#l'ordine è obj, variabili, vincoli
problem path: MC, 
#var
    wPath, 
    #conflict,
#constrains
    startingPath,finishingPath,allPath;
#    mergeConf1,diverConf1,diverConf2,splitConf1;


problem conflicts: UAM,
#variables
    t_ear, t_lat, t_ear_start,
    t_down, t_up, l,
#constrains
    afterprecalculated, calculateLat, startInt,
    limitT_down, limitT_up, 
    defineT_down, defineT_up,
    trail13, trail14,
    trail23, trail24,
    merge3, merge4,
    diver3, diver4,
    diver3_1, diver4_1,
    #diver3SameNode,diver3_1SameNode,
    #diver4SameNode,diver4_1SameNode,
    split3, split4;