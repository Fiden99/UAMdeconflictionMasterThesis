#data
param nf;																						#number of flights
param nn;																					#number of nodes
set F:= 0..nf-1;																				#set of flights
set V:= 0..nn-1;																				#set of nodes
set E within {V cross V};																#arcs
param s{F};																					#starting point for each flight
param e{F};																					#ending poitn for each flight
param d{x in V,y in V: (x,y) in E}>=0 default 0;											#distance for each pair of nodes
param v_max{F,x in V, y in V: (x,y) in E};
param v_min{F,x in V, y in V: (x,y) in E};
param dMCF{x in V,y in V: (x,y) in E} >=0 default 0;
#param v{F,V};																				#entering speed 
param bigM:=sum{f in F,i in V,j in V: (i,j) in E}d[i,j] * v_min[f,i,j];	#bigM for linearizing purpose
param angleM{x in V, x1 in V, x2 in V: (x1,x) in E and (x2,x) in E  and x1<> x2};							# angle-for merging
param angleP{x in V, x1 in V, x2 in V: (x,x1) in E and (x,x2) in E   and x1<>x2};							# angle+ for splitting
param anglePM{x in V, x1 in V,x2 in V: (x,x1) in E and (x2,x) in E and x1<>x2};							#angle -+ divering
param D;																						# safety distance
param t_hat_ear{F,V};
param t_hat_lat{F,V};
param w{i in V,j in V,F: (i,j) in E} binary;																		#flight f pass through arc i,j
param y1t{i in F, j in F, x in V, y in V:(x,y) in E and i<>j} binary;
param y2t{i in F, j in F, x in V, y in V:(x,y) in E and i<>j} binary;
param ym{i in F, j in F, x in V, x1 in V, x2 in V: (x1,x) in E and (x2,x) in E and i<>j and x1<> x2} binary;
param yd{i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x2,x) in E and  i<>j and x1<> x2} binary;
param ys{i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x,x2) in E and  i<>j and x1<> x2} binary;
#variables
var z_up{i in V,j in V,F: (i,j) in E} integer >=0 ;														# variable for w*t, understand why is not integer
var z_down{i in V,j in V,F: (i,j) in E} integer >=0;													#variable for w*t
var t_down{F,V} >=0;
var t_up {F,V} >= 0;
var t_ear{F,V} integer>=0 ;															#variable time, understand why is not integer
var t_lat{F,V} >=0;																		#variable time, undestand why is not integer
#heuristic var
var wPath{i in V,j in V,F: (i,j) in E} >=0, <=1;		

#binary variables for linearization of conflicts
var y1o1{i in F, j in F, x in V, y in V:(x,y) in E and i<>j} binary;
var y1o2{i in F, j in F, x in V, y in V:(x,y) in E and i<>j} binary;

var y2o1{i in F, j in F, x in V, y in V:(x,y) in E and i<>j} binary;
var y2o2{i in F, j in F, x in V, y in V:(x,y) in E and i<>j} binary;

var ymo1{i in F, j in F, x in V, x1 in V, x2 in V: (x1,x) in E and (x2,x) in E and i<>j and x1<> x2} binary;
var ymo2{i in F, j in F, x in V, x1 in V, x2 in V: (x1,x) in E and (x2,x) in E and i<>j and x1<> x2} binary;

var ydo1{i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x2,x) in E and  i<>j and x1<> x2} binary;
var ydo2{i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x2,x) in E and  i<>j and x1<> x2} binary;

var yso1{i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x,x2) in E and  i<>j and x1<> x2} binary;
var yso2{i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x,x2) in E and  i<>j and x1<> x2} binary;

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

subject to startingPath{i in F}:
sum{x in V: (x,s[i]) in E} wPath[x,s[i],i] -sum{x in V: (s[i],x) in E} wPath[s[i],x,i]=-1;
subject to finishingPath{i in F}:
sum{x in V: (x,e[i]) in E} wPath[x,e[i],i]- sum{x in V:(e[i],x) in E} wPath[e[i],x,i]=1;
subject to allPath{i in F, x in V :  x <> s[i] and x <> e[i]}:
sum{y in V: (x,y) in E} wPath[x,y,i]=sum{y in V: (y,x) in E} wPath[y,x,i];

subject to limitT_down{i in F, x in V : x <> s[i]}:
t_down[i,x] <= t_ear[i,x];
subject to limitT_up{i in F, x in V: x <> s[i]}:
t_ear[i,x] <= t_up[i,x];


subject to defineT_down{f in F,y in V}:	
t_down[f,y]=sum{x in V: (x,y) in E} (w[x,y,f] *d[x,y]/v_max[f,x,y] +  z_down[x,y,f]);

subject to linearizeDown1{f in F,x in V,y in V: (x,y) in E}:
z_down[x,y,f] <= bigM* w[x,y,f];
subject to linearizeDown2{f in F,x in V,y in V: (x,y) in E}:
z_down[x,y,f] <=t_ear[f,x];
subject to linearizeDown3{f in F,x in V,y in V: (x,y) in E}:
z_down[x,y,f] >=  t_ear[f,x] - bigM* (1- w[x,y,f]);

subject to defineT_up{f in F,y in V}: 		
t_up[f,y]=sum{x in V: (x,y) in E} (w[x,y,f] * d[x,y]/v_min[f,x,y] + z_up[x,y,f]);

subject to linearizeUp1{f in F,x in V,y in V: (x,y) in E}:
z_up[x,y,f] <= bigM* w[x,y,f];
subject to linearizeUp2{f in F,x in V,y in V: (x,y) in E}:
z_up[x,y,f] <=t_ear[f,x];
subject to linearizeUp3{f in F,x in V,y in V: (x,y) in E}:
z_up[x,y,f] >=  t_ear[f,x] - bigM* (1- w[x,y,f]);

# conflicts 

subject to trail11{i in F, j in F, x in V, y in V:(x,y) in E and i<>j}:
2*(1-y1t[i,j,x,y]) <= w[x,y,i]+w[x,y,j];
subject to trail12{i in F,j in F, x in V, y in V:(x,y) in E and i<>j}: 
w[x,y,i]+w[x,y,j] <= 2 -y1t[i,j,x,y];
subject to trail13 {i in F,j in F, x in V, y in V:(x,y) in E and i<>j}:
v_min[i,x,y]*(t_ear[j,x]-t_lat[i,x]) >= D-bigM*y1t[i,j,x,y] - y1o1[i,j,x,y]*bigM;
#v[i,y]*(t[j,x]-t[i,x]) >= D-bigM*y1t[i,j,x,y] - y1o1[i,j,x,y]*bigM;
subject to trail14 {i in F,j in F, x in V, y in V:(x,y) in E and i<>j}:
v_min[j,x,y]*(t_ear[i,x]-t_lat[j,x])>=D-bigM*y1t[i,j,x,y] - bigM*y1o2[i,j,x,y];
#v[j,y]*(t[i,x]-t[j,x])>=D-bigM*y1t[i,j,x,y] - bigM*y1o2[i,j,x,y];
subject to trail15 {i in F,j in F, x in V, y in V:(x,y) in E and i<>j}:
y1o1[i,j,x,y]+y1o2[i,j,x,y]<=1;

subject to trail21{i in F, j in F, x in V, y in V:(x,y) in E and i<>j}:
2*(1-y2t[i,j,x,y]) <= w[x,y,i]+w[x,y,j];
subject to trail22{i in F,j in F, x in V, y in V:(x,y) in E and i<>j}: 
w[x,y,i]+w[x,y,j] <= 2 -y2t[i,j,x,y];
subject to trail23 {i in F,j in F, x in V, y in V:(x,y) in E and i<>j}:
v_min[j,x,y]*(t_ear[j,y]-t_lat[i,y])>= D-bigM*y2t[i,j,x,y] - y2o1[i,j,x,y]*bigM;
#v[i,y]*(t[j,y]-t[i,y])>= D-bigM*y2t[i,j,x,y] - y2o1[i,j,x,y]*bigM;
subject to trail24 {i in F,j in F, x in V, y in V:(x,y) in E and i<>j}:
v_min[i,x,y]*(t_ear[i,y]-t_lat[j,y])>= D-bigM*y2t[i,j,x,y] - y2o2[i,j,x,y] * bigM;
#v[j,y]*(t[i,y]-t[j,y])>= D-bigM*y2t[i,j,x,y] - y2o2[i,j,x,y] * bigM;
subject to trail25 {i in F,j in F, x in V, y in V:(x,y) in E and i<>j}:
y2o1[i,j,x,y]+y2o2[i,j,x,y]<=1;

subject to merge1{i in F, j in F, x in V, x1 in V, x2 in V: (x1,x) in E and (x2,x) in E and i<>j and x1<> x2}:
2*(1-ym[i,j,x,x1,x2]) <= w[x1,x,i]+w[x2,x,j];
subject to merge2{i in F, j in F, x in V, x1 in V, x2 in V: (x1,x) in E and (x2,x) in E and i<>j and x1<> x2}: 
w[x1,x,i]+w[x2,x,j] <= 2 -ym[i,j,x,x1,x2];
#TODO check if the speed parameter is correct for merge4
subject to merge3 {i in F, j in F, x in V, x1 in V, x2 in V: (x1,x) in E and (x2,x) in E and  i<>j and x1<> x2}: 
t_ear[j,x]- t_lat[i,x]>=angleM[x,x1,x2]*D/v_min[j,x2,x]-bigM*ym[i,j,x,x1,x2] - ymo1[i,j,x,x1,x2]*bigM;
#t[j,x]- t[i,x]>=angleM[i,j,x]*D/v[j,x]-bigM*ym[i,j,x,x1,x2] - ymo1[i,j,x,x1,x2]*bigM;
subject to merge4 {i in F, j in F, x in V, x1 in V, x2 in V: (x1,x) in E and (x2,x) in E and  i<>j and x1<> x2}: 
t_ear[i,x]- t_lat[j,x]>=angleM[x,x1,x2]*D/v_min[i,x1,x]-bigM*ym[i,j,x,x1,x2] - ymo2[i,j,x,x1,x2]*bigM;
#t[i,x]- t[j,x]>=angleM[i,j,x]*D/v[i,x]-bigM*ym[i,j,x,x1,x2] - ymo2[i,j,x,x1,x2]*bigM;
subject to merge5{i in F, j in F, x in V, x1 in V, x2 in V: (x1,x) in E and (x2,x) in E and  i<>j and x1<> x2}:
ymo1[i,j,x,x1,x2]+ymo2[i,j,x,x1,x2]<=1;

subject to diver1 {i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x2,x) in E and  i<>j and x1<> x2}:
2*(1-yd[i,j,x,x1,x2]) <= w[x,x1,i]+w[x2,x,j];
subject to diver2 {i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x2,x) in E and  i<>j and x1<> x2}: 
w[x,x1,i]+w[x2,x,j] <= 2 -yd[i,j,x,x1,x2];
#unica maniera che ha senso
subject to diver3 {i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x2,x) in E and  i<>j and x1<> x2}: #and x <>e[i]
t_ear[j,x]- t_lat[i,x]>=anglePM[x,x1,x2]*(D/v_min[j,x2,x]+D/v_min[i,x,x1]) -bigM*yd[i,j,x,x1,x2] - ydo1[i,j,x,x1,x2]*bigM;
#t[j,x]- t[i,x]>=anglePM[i,j,x]*(D/v[j,x]+D/v[i,x1]) -bigM*yd[i,j,x,x1,x2] - ydo1[i,j,x,x1,x2]*bigM;
subject to diver4 {i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x2,x) in E and  i<>j and x1<> x2}: #and x <> e[j]
t_ear[i,x]- t_lat[j,x]>=anglePM[x,x1,x2]*(D/v_min[i,x2,x]+D/v_min[j,x,x1])-bigM*yd[i,j,x,x1,x2] - ydo2[i,j,x,x1,x2]*bigM;
#t[i,x]- t[j,x]>=anglePM[i,j,x]*(D/v[i,x]+D/v[j,x1])-bigM*yd[i,j,x,x1,x2] - ydo2[i,j,x,x1,x2]*bigM;
subject to diver5 {i in F, j in F, x in V, x1 in V, x2 in V:(x,x1) in E and (x2,x) in E and  i<>j and x1<> x2}:
ydo1[i,j,x,x1,x2]+ydo2[i,j,x,x1,x2]<=1;
#put ending cases


#change xy with explicit values that I have
subject to split1 {i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x,x2) in E and  i<>j and x1<> x2}:
2*(1-ys[i,j,x,x1,x2]) <= w[x,x1,i]+w[x,x2,j];
subject to split2 {i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x,x2) in E and  i<>j and x1<> x2}:
w[x,x1,i]+w[x,x2,j] <= 2 -ys[i,j,x,x1,x2];
#TODO check if the speed parameter is correct for split4
subject to split3 {i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x,x2) in E and  i<>j and x1<> x2}:
t_ear[j,x]- t_lat[i,x]>=angleP[x,x1,x2]*D/v_min[i,x,x1]-bigM*ys[i,j,x,x1,x2] - yso1[i,j,x,x1,x2]*bigM;
#t[j,x]- t[i,x]>=angleP[i,j,x]*D/v[i,x1]-bigM*ys[i,j,x,x1,x2] - yso1[i,j,x,x1,x2]*bigM;
subject to split4 {i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x,x2) in E and  i<>j and x1<> x2}:
t_ear[i,x]- t_lat[j,x]>=angleP[x,x1,x2]*D/v_min[j,x,x2]-bigM*ys[i,j,x,x1,x2] - yso2[i,j,x,x1,x2]*bigM;
#t[i,x]- t[j,x]>=angleP[i,j,x]*D/v[i,x2]-bigM*ys[i,j,x,x1,x2] - yso2[i,j,x,x1,x2]*bigM;
subject to split5 {i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x,x2) in E and  i<>j and x1<> x2}:
yso1[i,j,x,x1,x2]+yso2[i,j,x,x1,x2]<=1;

#objective 49
#minimize z: sum{i in F,x in V, y in V: (x,y) in E} w[x,y,i];
#objective 50
minimize UAM: sum{i in F} t_ear[i,e[i]];
minimize MC: sum{f in F, x in V,y in V: (x,y) in E} wPath[x,y,f] * dMCF[x,y];

#l'ordine è obj, variabili, vincoli
problem path: MC, wPath,startingPath,finishingPath,allPath;

problem conflicts: UAM,
#variables
    t_ear, t_lat, z_up, z_down, t_down, t_up,
    #y1t,y2t,ym, yd, ys,
    y1o1, y1o2,  y2o1, y2o2,  ymo1, ymo2, ydo1, ydo2, yso1, yso2,
    #w,
#constrains
    afterprecalculated, calculateLat, 
    #startingW, finishingW, allW,
    limitT_down, limitT_up, 
    defineT_down, linearizeDown1, linearizeDown2, linearizeDown3,
    defineT_up, linearizeUp1, linearizeUp2, linearizeUp3,
    trail11, trail12, trail13, trail14, trail15,
    trail21, trail22, trail23, trail24, trail25,
    merge1, merge2, merge3, merge4, merge5,
    diver1, diver2, diver3, diver4, diver5,
    split1, split2, split3, split4, split5;


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