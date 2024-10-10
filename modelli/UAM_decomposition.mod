#data
param nf integer;			#n of flight
param nn integer;			#n of nodes
set F:=1..nf;		#set of flight
set V:=1..nn;		#set of nodes
set E within {V cross V};	#arcs
set P{F} ordered;
param d{V,V}>=0;
param v_max{F,V};
param v_min{F,V};
param bigM:=sum{i in V,j in V}d[i,j];
param angleM{F,F,V};
param angleP{F,F,V};
param anglePM{F,F,V};
param D;
set trail within {F,F,V,V};
set merge within  {F,F,V};
set diver within {F,F,V};
set split within {F,F,V};
param t_hat_ear{F,V};
param t_hat_lat{F,V};
#var
var t_ear{F,V}>=0 integer;
var t_lat{F,V}>=0 integer;
var w{V,V,F} binary;
var t_up{F,V} >=0;		#tempo max per velocità
var t_down{F,V} >=0; 	#tempo min per velocità
var z_up{V,V,F} >=0 integer;					# replacement for w*t 
var z_down{V,V,F} >=0 integer;				# replacement for w*t 
#var zMax integer;

#binary var for linearize
var y1t{F,F,V,V} binary;
var y2t{F,F,V,V} binary;
var y1o1{F,F,V,V} binary;
var y1o2{F,F,V,V} binary;
var y2o1{F,F,V,V} binary;
var y2o2{F,F,V,V} binary;
var ym{F,F,V,V,V} binary;
var ymo1{F,F,V,V,V} binary;
var ymo2{F,F,V,V,V} binary;
var yd{F,F,V,V,V} binary;
var ydo1{F,F,V,V,V} binary;
var ydo2{F,F,V,V,V} binary;
var ys{F,F,V,V,V} binary;
var yso1{F,F,V,V,V} binary;
var yso2{F,F,V,V,V} binary;

#constrains

subject to afterprecalculated{f in F}:
t_hat_ear[f,first(P[f])]<= t_ear[f,first(P[f])];
subject to calculateLat{f in F, v in V}:
t_lat[f,v]=t_ear[f,v] + t_hat_lat[f,v] - t_hat_ear[f,v];

#subject to minSpeed{f in F, v in V} :
#t_down[f,v] <= t_ear[f,v];
#subject to maxSpeed{f in F,v in V}: 
#t_ear[f,v]<= t_up[f,v];

subject to startingW{f in F}: 
sum{x in V: (first(P[f]),x) in E} w[first(P[f]),x,f]=1;
subject to finishingW{f in F}:
sum{x in V: (x,last(P[f])) in E} w[x,last(P[f]),f]=1;
subject to allW{f in F,x in V diff {first(P[f]),last(P[f])}}:
sum{y in V: (x,y) in E} w[x,y,f]=sum{y in V: (x,y) in E} w[y,x,f];


#subject to defineT_down{f in F,y in V}:
#t_down[f,y]=sum{x in V: (x,y) in E} w[x,y,f]*(t_ear[f,x]+d[x,y]/v_max[f,y]);
#subject to defineT_up{f in F,y in V}:
#t_up[f,y]=sum{x in V: (x,y) in E} w[x,y,f]*(t_ear[f,x]+d[x,y]/v_min[f,x]);

subject to defineT_down{f in F,y in V}:	# valutare sostituzione di y in V con  in P[i] e e usare solo nodi che sono vicini a y
t_down[f,y]=sum{x in V: (x,y) in E} (w[x,y,f] *d[x,y]/v_max[f,y] +  z_down[x,y,f]);#t_ear[f,x]);

#subject to linearDown1{f in F,x in V,y in V}:
#t_ear[f,x]<=z_down[x,y,f];
#subject to linearDown2{f in F,x in V,y in V}:
#z_down[x,y,f] <= t_ear[f,x]* bigM;
subject to linearizeDown1{f in F,x in V,y in V}:
z_down[x,y,f] <= bigM* w[x,y,f];
subject to linearizeDown2{f in F,x in V,y in V}:
z_down[x,y,f] <=t_ear[f,x];
subject to linearizeDown3{f in F,x in V,y in V}:
z_down[x,y,f] >=  t_ear[f,x] - bigM* (1- w[x,y,f]);

subject to defineT_up{f in F,y in V}: 		
t_up[f,y]=sum{x in V: (x,y) in E} (w[x,y,f] * d[x,y]/v_min[f,y] + z_up[x,y,f]);

#subject to linearUp1{f in F,x in V,y in V}:
#t_ear[f,x]<=z_up[x,y,f];
#subject to linearUp2{f in F,x in V,y in V}:
#z_up[x,y,f] <=t_ear[f,x]* bigM;
subject to linearizeUp1{f in F,x in V,y in V}:
z_up[x,y,f] <= bigM* w[x,y,f];
subject to linearizeUp2{f in F,x in V,y in V}:
z_up[x,y,f] <=t_ear[f,x];
subject to linearizeUp3{f in F,x in V,y in V}:
z_up[x,y,f] >=  t_ear[f,x] - bigM* (1- w[x,y,f]);


#subject to trail1{i in F, j in F, (x,y) in E}:
#(w[x,y,i]+w[x,y,j]=2)==> (v_min[i,y]*(t_ear[j,x]-t_lat[i,x]>=D) or (v_min[j,y]*(t_ear[i,x]-t_lat[j,x]>=D));

subject to trail11{i in F, j in F, (x,y) in E}:
2*(1-y1t[i,j,x,y]) <= w[x,y,i]+w[x,y,j];
subject to trail12{i in F,j in F, (x,y) in E}: 
w[x,y,i]+w[x,y,j] <= 2 -y1t[i,j,x,y];

#subject to trail13{ i in F,j in F, (x,y) in E}:
#(v_min[i,y]*(t_ear[j,x]-t_lat[i,x])>=D-bigM*y1t) or (v_min[j,y]*(t_ear[i,x]-t_lat[j,x])>=D-bigM*y1t);

subject to trail13 {i in F,j in F, (x,y) in E: (i,j,x,y) in trail}:
v_min[i,y]*(t_ear[j,x]-t_lat[i,x]) >= D-bigM*y1t[i,j,x,y] - y1o1[i,j,x,y]*bigM;
subject to trail14 {i in F,j in F, (x,y) in E: (i,j,x,y) in trail}:
v_min[j,y]*(t_ear[i,x]-t_lat[j,x])>=D-bigM*y1t[i,j,x,y] - bigM*y1o2[i,j,x,y];
subject to trail15 {i in F,j in F, (x,y) in E:  (i,j,x,y) in trail}:
y1o1[i,j,x,y]+y1o2[i,j,x,y]<=1;

#(w[x,y,i]+w[x,y,j]=2)==> (v_min[i,y]*(t_ear[j,x]-t_lat[i,x])>=D) or (v_min[j,y]*(t_ear[i,x]-t_lat[j,x]>=D));


#subject to trail2{i in F, j in F, (x,y) in E}:
#(w[x,y,i]+w[x,y,j]=2)==> ((v_min[i,y]*(t_ear[j,y]-t_lat[i,y])>=D) or (v_min[j,y]*(t_ear[i,y]-t_lat[j,y])>=D));

subject to trail21{i in F, j in F, (x,y) in E}:
2*(1-y2t[i,j,x,y]) <= w[x,y,i]+w[x,y,j];
subject to trail22{i in F,j in F, (x,y) in E}: 
w[x,y,i]+w[x,y,j] <= 2 -y2t[i,j,x,y];

subject to trail23 {i in F,j in F, (x,y) in E: (i,j,x,y) in trail}:
v_min[i,y]*(t_ear[j,y]-t_lat[i,y])>= D-bigM*y2t[i,j,x,y] - y2o1[i,j,x,y]*bigM;
subject to trail24 {i in F,j in F, (x,y) in E: (i,j,x,y) in trail}:
v_min[j,y]*(t_ear[i,y]-t_lat[j,y])>= D-bigM*y2t[i,j,x,y] - y2o2[i,j,x,y] * bigM;
subject to trail25 {i in F,j in F, (x,y) in E: (i,j,x,y) in trail}:
y2o1[i,j,x,y]+y2o2[i,j,x,y]<=1;



#subject to merge{i in F, j in F, x1 in V, x2 in V, y in V: (x1,y) in E,(x2,y) in E}:
#(w[x1,y,i]+w[x2,y,j]=2)==> ((t_ear[j,y]- t_lat[i,y]>=angleM[i,j,y]*D/v_min[j,y]) or (t_ear[i,y]- t_lat[j,y]>=angleM[i,j,y]*D/v_min[i,y]));
#to check if x is x', so y

subject to merge1{i in F, j in F, x in V, x1 in V, x2 in V: (x1,x) in E and (x2,x) in E}:
2*(1-ym[i,j,x,x1,x2]) <= w[x1,x,i]+w[x2,x,j];
subject to merge2{i in F, j in F, x in V, x1 in V, x2 in V: (x1,x) in E and (x2,x) in E}: 
w[x1,x,i]+w[x2,x,j] <= 2 -ym[i,j,x,x1,x2];
subject to merge3 {i in F, j in F, x in V, x1 in V, x2 in V: (x1,x) in E and (x2,x) in E and (i,j,x) in merge}: 
t_ear[j,x]- t_lat[i,x]>=angleM[i,j,x]*D/v_min[j,x]-bigM*ym[i,j,x,x1,x2] - ymo1[i,j,x,x1,x2]*bigM;
subject to merge4 {i in F, j in F, x in V, x1 in V, x2 in V: (x1,x) in E and (x2,x) in E and (i,j,x) in merge}: 
t_ear[i,x]- t_lat[j,x]>=angleM[i,j,x]*D/v_min[i,x]-bigM*ym[i,j,x,x1,x2] - ymo2[i,j,x,x1,x2]*bigM;
subject to merge5{i in F, j in F, x in V, x1 in V, x2 in V: (x1,x) in E and (x2,x) in E and (i,j,x) in merge}:
ymo1[i,j,x,x1,x2]+ymo2[i,j,x,x1,x2]<=1;


#subject to diver{i in F, j in F, x1 in V, x2 in V, y in V: (y,x1) in E,(x2,y) in E}:
#(w[y,x1,i]+w[x2,y,j]=2)==> ((t_ear[j,y]- t_lat[i,x]>=anglePM[i,j,y]*(D/v_min[j,y]+D/v_min[i,next(x,P[i],+1)])) or (t_ear[i,y]- t_lat[j,y]>=anglePM[i,j,y]*(D/v_min[i,y]+D/v_min[j,next(x,P[j],+1)])));

subject to diver1 {i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x2,x) in E}:
2*(1-yd[i,j,x,x1,x2]) <= w[x,x1,i]+w[x2,x,j];
subject to diver2 {i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x2,x) in E}: 
w[x,x1,i]+w[x2,x,j] <= 2 -yd[i,j,x,x1,x2];
subject to diver3 {i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x2,x) in E and (i,j,x) in diver}: 
t_ear[j,x]- t_lat[i,x]>=anglePM[i,j,x]*(D/v_min[j,x]+D/v_min[i,x1]) -bigM*yd[i,j,x,x1,x2] - ydo1[i,j,x,x1,x2]*bigM;
#t_ear[j,x]- t_lat[i,x]>=anglePM[i,j,x]*(D/v_min[j,x]+D/v_min[i,next(x,P[i],+1)]) -bigM*yd[i,j,x,x1,x2] - ydo1[i,j,x,x1,x2]*bigM;
subject to diver4 {i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x2,x) in E and (i,j,x) in diver}: 
t_ear[i,x]- t_lat[j,x]>=anglePM[i,j,x]*(D/v_min[i,x]+D/v_min[j,x2])-bigM*yd[i,j,x,x1,x2] - ydo2[i,j,x,x1,x2]*bigM;
#t_ear[i,x]- t_lat[j,x]>=anglePM[i,j,x]*(D/v_min[i,x]+D/v_min[j,next(x,P[j],+1)])-bigM*yd[i,j,x,x1,x2] - ydo2[i,j,x,x1,x2]*bigM;
subject to diver5 {i in F, j in F, x in V, x1 in V, x2 in V:(x,x1) in E and (x2,x) in E and (i,j,x) in diver}:
ydo1[i,j,x,x1,x2]+ydo2[i,j,x,x1,x2]<=1;


#subject to split{i in F, j in F, x1 in V, x2 in V, y in V: (y,x1) in E,(y,x2) in E}:
#(w[y,x1,i]+w[y,x2,j]=2)==> ((t_ear[j,y]- t_lat[i,y]>=angleP[i,j,y]*D/v_min[i,next(x,P[i],+1)]) or (t_ear[i,y]- t_lat[j,y]>=angleP[i,j,y]*D/v_min[j,next(x,P[j],+1)]));

#change xy with explicit values that I have
subject to split1 {i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x,x2) in E}:
2*(1-ys[i,j,x,x1,x2]) <= w[x,x1,i]+w[x,x2,j];
subject to split2 {i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x,x2) in E}:
w[x,x1,i]+w[x,x2,j] <= 2 -ys[i,j,x,x1,x2];
subject to split3 {i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x,x2) in E and (i,j,x) in split}: # and x!=last(P,i)
t_ear[j,x]- t_lat[i,x]>=angleP[i,j,x]*D/v_min[i,x1]-bigM*ys[i,j,x,x1,x2] - yso1[i,j,x,x1,x2]*bigM;
#t_ear[j,x]- t_lat[i,x]>=angleP[i,j,x]*D/v_min[i,next(x,P[i],+1)]-bigM*ys[i,j,x,x1,x2] - yso1[i,j,x,x1,x2]*bigM;
subject to split4 {i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x,x2) in E and (i,j,x) in split}:
t_ear[i,x]- t_lat[j,x]>=angleP[i,j,x]*D/v_min[j,x2]-bigM*ys[i,j,x,x1,x2] - yso2[i,j,x,x1,x2]*bigM;
#t_ear[i,x]- t_lat[j,x]>=angleP[i,j,x]*D/v_min[j,next(x,P[j],+1)]-bigM*ys[i,j,x,x1,x2] - yso2[i,j,x,x1,x2]*bigM;
subject to split5 {i in F, j in F, x in V, x1 in V, x2 in V: (x,x1) in E and (x,x2) in E and  (i,j,x) in split}:
yso1[i,j,x,x1,x2]+yso2[i,j,x,x1,x2]<=1;


#understand obj function
#minimize z: zMax;