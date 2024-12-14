#data
param nf;																						#number of flights
param nn;																					#number of nodes
set F:= 0..nf-1;																				#set of flights
set V:= 0..nn-1;																				#set of nodes
set E within {V cross V};																#arcs
param s{F};																					#starting point for each flight
param e{F};																					#ending poitn for each flight
param d{E}>=0 default 0;											                    #distance for each pair of nodes
param v_max{F,E};
param v_min{F,E};
param dMCF{E} >=0 default 0;
#param v{F,V};																				#entering speed 
param bigM:= 1000;	#bigM for linearizing purpose
param angleM{x in V, (x1,x) in E, (x2,x) in E: x1<>x2};							# angle-for merging
param angleP{x in V, (x,x1) in E, (x,x2) in E: x1<>x2};							# angle+ for splitting
param anglePM{x in V,(x,x1) in E, (x2,x) in E: x1<>x2};							#angle -+ divering
param D;																						# safety distance
param t_hat_ear{F,V};
param t_hat_lat{F,V};
param w{(i,j) in E,F} binary;																		#flight f pass through arc i,j
param y1t{i in F, j in F, (x,y) in E: i<>j} binary default 1;
param y2t{i in F, j in F, (x,y) in E: i<>j} binary default 1;
param ym{i in F, j in F, x in V,(x1,x) in E, (x2,x) in E: i<>j and x1<>x2} binary default 1;
param yd{i in F, j in F, x in V,(x,x1) in E, (x2,x) in E: i<>j and x1<>x2} binary default 1;
param ys{i in F, j in F, x in V,(x,x1) in E, (x,x2) in E: i<>j and x1<>x2} binary default 1;
#variables
var z_up{(i,j) in E,F} integer >=0 ;													#variable for w*t, understand why is not integer
var z_down{(i,j) in E, F} integer >=0;													#variable for w*t
var t_down{F,V} >=0;
var t_up {F,V} >= 0;
var t_ear{F,V} integer>=0 ;															    #variable time, understand why is not integer
var t_lat{F,V} >=0;																		#variable time, undestand why is not integer
#heuristic var
var wPath{(i,j) in E, F} >=0, <=1;		
var wConf{(i,j) in E, F} binary;


var numConflicts{(x,y) in E,i in F, j in F : i<> j} >= 0, <=1;

#var l{i in F, j in F, x in V: i<>j and ((sum{(x,x1) in E} w[x,x1,i] ==  1 and sum{(x,x1) in E} w[x,x1,j] == 1) or (sum{(x1,x) in E} w[x1,x,i] ==  1 and sum{(x1,x) in E} w[x1,x,j] == 1 ) or (sum{(x1,x) in E} w[x1,x,i] ==  1 and sum{(x,x1) in E} w[x,x1,j] == 1))} binary;
#var l{i in F, j in F, x in V: i<>j and sum{(x,x1) in E} w[x,x1,i] ==  1 and sum{(x,x1) in E} w[x,x1,j] == 1 } binary;
var l{i in F, j in F, x in V: i<>j } binary;

#constrains

subject to afterprecalculated{f in F}:
t_hat_ear[f,s[f]]<= t_ear[f,s[f]];
subject to calculateLat{i in F, x in V}:
t_lat[i,x]=t_ear[i,x] + t_hat_lat[i,x] - t_hat_ear[i,x];

subject to startingW{i in F}:
sum{x in V: (s[i],x) in E} w[s[i],x,i]=1;
subject to finishingW{i in F}:
sum{x in V: (x,e[i]) in E} w[x,e[i],i]=1;
subject to allW{i in F, x in V :  x <> s[i] and x <> e[i]}:
sum{y in V: (x,y) in E} w[x,y,i]=sum{y in V: (y,x) in E} w[y,x,i];

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

subject to startingPath{i in F}:
sum{(x,s[i]) in E} wPath[x,s[i],i] -sum{(s[i],x) in E} wPath[s[i],x,i]=-1;
subject to finishingPath{i in F}:
sum{(x,e[i]) in E} wPath[x,e[i],i]- sum{(e[i],x) in E} wPath[e[i],x,i]=1;
subject to allPath{i in F, x in V :  x <> s[i] and x <> e[i]}:
sum{(x,y) in E} wPath[x,y,i]=sum{(y,x) in E} wPath[y,x,i];

subject to limitT_down{i in F, x in V : x <> s[i]}:
t_down[i,x] <= t_ear[i,x];
subject to limitT_up{i in F, x in V: x <> s[i]}:
t_ear[i,x] <= t_up[i,x];


subject to defineT_down{f in F,y in V}:	
t_down[f,y]=sum{(x,y) in E} (w[x,y,f] *d[x,y]/v_max[f,x,y] +  z_down[x,y,f]);

subject to linearizeDown1{f in F,(x,y) in E}:
z_down[x,y,f] <= bigM* w[x,y,f];
subject to linearizeDown2{f in F,(x,y) in E}:
z_down[x,y,f] <=t_ear[f,x];
subject to linearizeDown3{f in F,(x,y) in E}:
z_down[x,y,f] >=  t_ear[f,x] - bigM* (1- w[x,y,f]);

subject to defineT_up{f in F,y in V}: 		
t_up[f,y]=sum{x in V: (x,y) in E} (w[x,y,f] * d[x,y]/v_min[f,x,y] + z_up[x,y,f]);

subject to linearizeUp1{f in F, (x,y) in E}:
z_up[x,y,f] <= bigM* w[x,y,f];
subject to linearizeUp2{f in F, (x,y) in E}:
z_up[x,y,f] <=t_ear[f,x];
subject to linearizeUp3{f in F, (x,y) in E}:
z_up[x,y,f] >=  t_ear[f,x] - bigM* (1- w[x,y,f]);

# conflicts 
/*
var l{i in F, j in F, x in V: i<>j and sum{(x,x1) in E} w[x,x1,i] ==  1 and sum{(x,x1) in E} w[x,x1,j] == 1 } binary;
# l mi indica se nel nodo x i passa prima di j 
subject to trail13{i in F,j in F,(x,y) in E: i<>j and w[x,y,i]+w[x,y,j]==2}:
t_ear[j,x]-t_lat[i,x] >= D/v_min[i,x,y]* l[i,j,x] - bigM*(1-l[i,j,x]);

subject to trail14 {i in F, j in F,(x,y) in E: i<>j and w[x,y,i]+w[x,y,j]==2}:
t_ear[i,x]-t_lat[j,x] >=D/v_min[j,x,y] * (1-l[i,j,x]) - bigM*l[i,j,x];

trail23 e trail24

subject to split4 {i in F, j in F,x in V,(x,x1) in E, (x,x2) in E: i<>j and x1<>x2}:
t_ear[i,x]- t_lat[j,x]>=angleP[x,x1,x2]*D/v_min[j,x,x2]* l[i,j,x] - bigM*(1-l[i,j,x]);
*/

subject to trail13 {i in F, j in F,(x,y) in E: i<>j and w[x,y,i]+w[x,y,j]==2}:
t_ear[j,x]-t_lat[i,x] >= D/v_min[i,x,y] * l[i,j,x ]- bigM*(1-l[i,j,x]);
#v[i,y]*(t[j,x]-t[i,x]) >= D-bigM*y1t[i,j,x,y] - y1o1[i,j,x,y]*bigM;
subject to trail14 {i in F, j in F,(x,y) in E: i<>j and w[x,y,i]+w[x,y,j]==2}:
t_ear[i,x]-t_lat[j,x] >=D/v_min[j,x,y] * (1-l[i,j,x]) - bigM*l[i,j,x];
#v[j,y]*(t[i,x]-t[j,x])>=D-bigM*y1t[i,j,x,y] - bigM*y1o2[i,j,x,y];



#capire se qui mi devo riferire al nodo x o al nodo y
subject to trail23 {i in F, j in F,(x,y) in E: i<>j and w[x,y,i]+w[x,y,j]==2}:
t_ear[j,y]-t_lat[i,y]>= D/v_min[j,x,y]* l[i,j,x] - bigM*(1-l[i,j,x]);
#v[i,y]*(t[j,y]-t[i,y])>= D-bigM*y2t[i,j,x,y] - y2o1[i,j,x,y]*bigM;
subject to trail24 {i in F, j in F,(x,y) in E: i<>j and w[x,y,i]+w[x,y,j]==2}:
t_ear[i,y]-t_lat[j,y]>= D/v_min[i,x,y] * (1-l[i,j,x]) - bigM*l[i,j,x];
#v[j,y]*(t[i,y]-t[j,y])>= D-bigM*y2t[i,j,x,y] - y2o2[i,j,x,y] * bigM;


#TODO check if the speed parameter is correct for merge4
subject to merge3 {i in F, j in F,x in V,(x1,x) in E, (x2,x) in E: i<>j and x1<>x2 and w[x1,x,i]+w[x2,x,j]==2}: 
t_ear[j,x]- t_lat[i,x]>=angleM[x,x1,x2]*D/v_min[j,x2,x] * l[i,j,x] - bigM*(1-l[i,j,x]);
#t[j,x]- t[i,x]>=angleM[i,j,x]*D/v[j,x]-bigM*ym[i,j,x,x1,x2] - ymo1[i,j,x,x1,x2]*bigM;
subject to merge4 {i in F, j in F,x in V,(x1,x) in E, (x2,x) in E: i<>j and x1<>x2 and w[x1,x,i]+w[x2,x,j]==2}: 
t_ear[i,x]- t_lat[j,x]>=angleM[x,x1,x2]*D/v_min[i,x1,x]* (1-l[i,j,x]) - bigM*l[i,j,x];

subject to diver3 {i in F, j in F,x in V,(x,x1) in E, (x2,x) in E: i<>j and x1<>x2 and w[x,x1,i] + w[x2,x,j]==2}: #and x <>e[i]
t_ear[j,x]- t_lat[i,x]>=anglePM[x,x1,x2]*(D/v_min[j,x2,x]+D/v_min[i,x,x1])*l[i,j,x] - bigM*(1-l[i,j,x]);
#t[j,x]- t[i,x]>=anglePM[i,j,x]*(D/v[j,x]+D/v[i,x1]) -bigM*yd[i,j,x,x1,x2] - ydo1[i,j,x,x1,x2]*bigM;
subject to diver4 {i in F, j in F,x in V,(x,x1) in E, (x2,x) in E: i<>j and x1<>x2 and w[x,x1,i] + w[x2,x,j]==2}: #and x <> e[j]
t_ear[i,x]- t_lat[j,x]>=anglePM[x,x1,x2]*(D/v_min[i,x2,x]+D/v_min[j,x,x1])* (1-l[i,j,x]) - bigM*l[i,j,x];
#t[i,x]- t[j,x]>=anglePM[i,j,x]*(D/v[i,x]+D/v[j,x1])-bigM*yd[i,j,x,x1,x2] - ydo2[i,j,x,x1,x2]*bigM;
#put ending cases


#change xy with explicit values that I have
#TODO check if the speed parameter is correct for split4
subject to split3 {i in F, j in F,x in V,(x,x1) in E, (x,x2) in E: i<>j and x1<>x2 and w[x,x1,i]+w[x,x2,j]==2}:
t_ear[j,x]- t_lat[i,x]>=angleP[x,x1,x2]*D/v_min[i,x,x1]*l[i,j,x] - bigM*(1-l[i,j,x]);
#t[j,x]- t[i,x]>=angleP[i,j,x]*D/v[i,x1]-bigM*ys[i,j,x,x1,x2] - yso1[i,j,x,x1,x2]*bigM;
subject to split4 {i in F, j in F,x in V,(x,x1) in E, (x,x2) in E: i<>j and x1<>x2 and w[x,x1,i]+w[x,x2,j]==2}:
t_ear[i,x]- t_lat[j,x]>=angleP[x,x1,x2]*D/v_min[j,x,x2] * (1-l[i,j,x]) - bigM*l[i,j,x];
#t[i,x]- t[j,x]>=angleP[i,j,x]*D/v[i,x2]-bigM*ys[i,j,x,x1,x2] - yso2[i,j,x,x1,x2]*bigM;




#objective 49
#minimize z: sum{i in F,x in V, y in V: (x,y) in E} w[x,y,i];
#objective 50
minimize UAM: sum{i in F} t_ear[i,e[i]];
minimize MC: sum{f in F, (x,y) in E} wPath[x,y,f] * dMCF[x,y];
minimize MinConf: sum{i in F,j in F,(x,y) in E: i<>j} numConflicts[x,y,i,j];

#l'ordine è obj, variabili, vincoli
problem path: MC, wPath,startingPath,finishingPath,allPath;

problem minCon: MinConf, wConf,numConflicts,startingPathConf,finishingPathConf,allPathConf,definenumConf1,definenumConf2,definenumConf3;


#TOOD remove trail12, trail11, trail21, trail22, merge1, merge2, split1, split2, diver1, diver2
problem conflicts: UAM,
#variables
    t_ear, t_lat, z_up, z_down, t_down, t_up, l,
    #y1t,y2t,ym, yd, ys,
    #w,
#constrains
    afterprecalculated, calculateLat, 
    #startingW, finishingW, allW,
    limitT_down, limitT_up, 
    defineT_down, linearizeDown1, linearizeDown2, linearizeDown3,
    defineT_up, linearizeUp1, linearizeUp2, linearizeUp3,
    #trail11, trail12, 
    trail13, trail14,
    #trail21, trail22, 
    trail23, trail24,
    #merge1, merge2, 
    merge3, merge4,
    #diver1, diver2, 
    diver3, diver4,
    #split1, split2, 
    split3, split4;

/*
problem wholeModel: UAM,
#variables
    #w,
    t_ear, t_lat, z_up, z_down, t_down, t_up,
    #y1t,y2t,ym, yd, ys,
    y1o1, y1o2,  y2o1, y2o2,  ymo1, ymo2, ydo1, ydo2, yso1, yso2,
#constrains
    afterprecalculated, calculateLat, 
    startingW, finishingW, allW,
    limitT_down, limitT_up, 
    defineT_down, linearizeDown1, linearizeDown2, linearizeDown3,
    defineT_up, linearizeUp1, linearizeUp2, linearizeUp3,
    trail11, trail12, trail13, trail14, trail15,
    trail21, trail22, trail23, trail24, trail25,
    merge1, merge2, merge3, merge4, merge5,
    diver1, diver2, diver3, diver4, diver5,
    split1, split2, split3, split4, split5;
*/
/*
subject to trail1{i in F,j in F, x in V, y in V:(x,y) in E}:
(w[x,y,i]+ w[x,y,j]=2) ==> ((v[i,y]*(t[j,x]-t[i,x])>= D) or (v[j,y]*(t[i,x]-t[j,x]) >=D));
subject to trail2{i in F,j in F, x in V, y in V:(x,y) in E}:
(w[x,y,i]+ w[x,y,j]=2) ==> ((v[i,y]*(t[j,y]-t[i,y])>= D) or (v[j,y]*(t[i,y]-t[j,y]) >=D));
subject to merge{i in F,j in F, x in V, x1 in V, x2 in V:(x1,x) in E and (x2,x) in E}:
(w[1,x,i]+ w[x2,x,j]=2) ==> ((t[j,x]-t[i,x]>=angleM[i,j,x]* D/v[j,x]) or (t[i,x]-t[j,x] >= angleM[i,j,x]*D/v[i,x]));
subject to diver{i in F,j in F, x in V, x1 in V, x2 in V:(x,x1) in E and (x2,x) in E}:
(w[x,x1,i]+ w[x2,x,j]=2) ==> ((t[j,x]-t[i,x]>= anglePM[i,j,x]*(D/v[j,x]+ D/v[i,x1])) or (t[i,x]-t[j,x] >= anglePM[i,j,x]*(D/v[i,x]+ D/v[j,x1])));
subject to split{i in F,j in F, x in V, x1 in V, x2 in V:(x,x1) in E and (x,x2) in E}:
(w[x,x1,i]+ w[x,x2,j]=2) ==> ((t[j,x]-t[i,x]>= angleP[i,j,x]* D/v[i,x1]) or (t[i,x]-t[j,x] >=angleP[i,j,x]*D/v[j,x2]));
*/