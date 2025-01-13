#data
param nn; #number of nodes
set F; #set of flights
set V:= 0..nn-1; #set of nodes
set E within {V cross V}; #arcs
param s{F}; #starting point for each flight
param e{F}; #ending poitn for each flight
param d{E}>=0; #distance for each pair of nodes
param v_max;
param v_min; #param v{F,V}; #entering speed
param bigM:=1000; #bigM for linearizing purpose
set conflictsNodes within {x in V, x1 in V, x2 in V: x1<>x2 and ((x,x1) in E or (x1,x) in E) and ((x,x2) in E or (x2,x) in E)};
param angle{conflictsNodes} ; #angle -+ divering
param D; # safety distance
param t_hat_ear{F,V} default 997;
param t_hat_lat{F,V} default 999;
param drifted_flight default -1;
param drifted_wp default -1;
param drifted_t_ear_fix  default -1;
param drifted_t_lat_fix  default -1;
set fixedFlights within {F cross E};


#capire come ottenere solo i voli fissati
set fixedF = setof {(f,x,y) in fixedFlights} f;

set fixedToZero := {f in fixedF,(x,y) in E: (f,x,y) not in fixedFlights};
#variables
var w{E,F} binary; #flight f pass through arc i,j
var t_down{F,V} >=0;
var t_up {F,V} >= 0;
var t_ear{F,V} >=0 ; #variable time, understand why is not integer
var t_lat{F,V} >=0; #variable time, undestand why is not integer
var t_ear_start{F} integer >=0;
var abs_t_ear{F} >=0;

#binary variables for linearization of conflicts
var passFirst{i in F, j in F, x in V: i<>j} binary;

#constrains
subject to startInteger{f in F}:
t_ear[f,s[f]] = t_ear_start[f];

subject to afterprecalculated{f in F}:
t_hat_ear[f,s[f]] <= t_ear_start[f];
subject to calculateLat{i in F, x in V}:
t_lat[i,x]=t_ear[i,x] + t_hat_lat[i,x] - t_hat_ear[i,x];

subject to startingW{i in F}:
sum{(x,s[i]) in E} w[x,s[i],i] - sum{(s[i],x) in E} w[s[i],x,i]=-1;
subject to finishingW{i in F}:
sum{(x,e[i]) in E} w[x,e[i],i] - sum{(e[i],x) in E} w[e[i],x,i]=1;
subject to allW{i in F,x in V : x<> s[i] and x <> e[i]}:
sum{(x,y) in E} w[x,y,i]=sum{(y,x) in E} w[y,x,i];

subject to limitT_down{i in F, x in V : x <> s[i]}:
t_down[i,x] <= t_ear[i,x];
subject to limitT_up{i in F, x in V: x <> s[i]}:
t_ear[i,x] <= t_up[i,x];


subject to defineT_down{f in F,y in V: y <> s[f]}:
t_down[f,y]=sum{(x,y) in E} w[x,y,f] *(d[x,y]/v_max + t_ear[f,x]);
subject to defineT_up{f in F,y in V: y <> s[f]}:
t_up[f,y]=sum{(x,y) in E} w[x,y,f] * (d[x,y]/v_min + t_ear[f,x]);

# conflicts


subject to trail13 {i in F,j in F, (x,y) in E: i<>j and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x,y,i] + w[x,y,j] == 2) ==> (t_ear[j,x]-t_lat[i,x] >= D/v_min-bigM* passFirst[i,j,x]);
#v[i,y]*(t[j,x]-t[i,x]) >= D-bigM*y1t[i,j,x,y] - y1o1[i,j,x,y]*bigM;
subject to trail14 {i in F,j in F, (x,y) in E: i<>j and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x,y,i] + w[x,y,j] == 2) ==> (t_ear[i,x]-t_lat[j,x]>=D/v_min* (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);
#v[j,y]*(t[i,x]-t[j,x])>=D-bigM*y1t[i,j,x,y] - bigM*y1o2[i,j,x,y];

subject to trail23 {i in F,j in F, (x,y) in E: i<>j and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
(w[x,y,i] + w[x,y,j] == 2) ==> t_ear[j,y]-t_lat[i,y]>= D/v_min* passFirst[i,j,x] - bigM*(1-passFirst[i,j,x]);
#v[i,y]*(t[j,y]-t[i,y])>= D-bigM*y2t[i,j,x,y] - y2o1[i,j,x,y]*bigM;
subject to trail24 {i in F,j in F, (x,y) in E: i<>j and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
(w[x,y,i] + w[x,y,j] == 2) ==> t_ear[i,y]-t_lat[j,y]>= D/v_min * (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x];
#v[j,y]*(t[i,y]-t[j,y])>= D-bigM*y2t[i,j,x,y] - y2o2[i,j,x,y] * bigM;

#undestand if (w[x,y,i] + w[x,y,j] == 2) ==> is enough, or if we have to consider the opposite (w[x,y,j] + w[x,y,i] == 2)
#tried with  (w[x1,x,i]+w[x2,x,j] +w[x1,x,j]+w[x2,x,i] >= 2) check if it is too much

#TODO check if the speed parameter is correct for merge4
subject to merge3 {i in F, j in F, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x1,x,i]+w[x2,x,j] == 2) ==> (t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min * passFirst[i,j,x] - bigM*(1-passFirst[i,j,x]));
subject to merge3_1 {i in F, j in F, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x1,x,j]+w[x2,x,i] == 2) ==> (t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min * passFirst[i,j,x] - bigM*(1-passFirst[i,j,x]));
#t[j,x]- t[i,x]>=angleM[i,j,x]*D/v[j,x]-bigM*ym[i,j,x,x1,x2] - ymo1[i,j,x,x1,x2]*bigM;
subject to merge4 {i in F, j in F, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and t_hat_ear[i,x] < t_hat_ear[j,x]}:
(w[x1,x,j]+w[x2,x,i] == 2) ==> (t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min* (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);
subject to merge4_1 {i in F, j in F, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and t_hat_ear[i,x] < t_hat_ear[j,x]}:
(w[x1,x,i]+w[x2,x,j] == 2) ==> (t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min* (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);
#t[i,x]- t[j,x]>=angleM[i,j,x]*D/v[i,x]-bigM*ym[i,j,x,x1,x2] - ymo2[i,j,x,x1,x2]*bigM;


subject to diver3 {i in F, j in F, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
(w[x,x1,i]+w[x2,x,j] == 2) ==> (t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)*passFirst[i,j,x] - bigM*(1-passFirst[i,j,x])) ;
subject to diver3_1 {i in F, j in F, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
(w[x,x1,j]+w[x2,x,i] == 2) ==> (t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)*passFirst[i,j,x] - bigM*(1-passFirst[i,j,x])) ;
#t[j,x]- t[i,x]>=anglePM[i,j,x]*(D/v[j,x]+D/v[i,x1]) -bigM*yd[i,j,x,x1,x2] - ydo1[i,j,x,x1,x2]*bigM;
subject to diver4 {i in F, j in F, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
(w[x,x1,i]+w[x2,x,j] == 2) ==> (t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)* (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);
subject to diver4_1 {i in F, j in F, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
(w[x,x1,j]+w[x2,x,i] == 2) ==> (t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)* (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);

#t[i,x]- t[j,x]>=anglePM[i,j,x]*(D/v[i,x]+D/v[j,x1])-bigM*yd[i,j,x,x1,x2] - ydo2[i,j,x,x1,x2]*bigM;


#TODO check if the speed parameter is correct for split4
subject to split3 {i in F, j in F, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x,x1,i]+w[x,x2,j] == 2) ==> (t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min*passFirst[i,j,x] - bigM*(1-passFirst[i,j,x]));
subject to split3_1 {i in F, j in F, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x,x1,j]+w[x,x2,i] == 2) ==> (t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min*passFirst[i,j,x] - bigM*(1-passFirst[i,j,x]));
#t[j,x]- t[i,x]>=angleP[i,j,x]*D/v[i,x1]-bigM*ys[i,j,x,x1,x2] - yso1[i,j,x,x1,x2]*bigM;
subject to split4 {i in F, j in F, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x,x1,i]+w[x,x2,j] == 2) ==> (t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min * (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);
subject to split4_1 {i in F, j in F, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x,x1,j]+w[x,x2,i] == 2) ==> (t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min * (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);
#t[i,x]- t[j,x]>=angleP[i,j,x]*D/v[i,x2]-bigM*ys[i,j,x,x1,x2] - yso2[i,j,x,x1,x2]*bigM;

#added constraints for drifted flight
subject to driftEar:
t_ear[drifted_flight,drifted_wp] >= drifted_t_ear_fix;
subject to driftLat:
t_ear[drifted_flight,drifted_wp] <= drifted_t_lat_fix;
subject to driftWP:
sum{(x,drifted_wp) in E} w[x,drifted_wp,drifted_flight] = 1;

subject to fixFlights{(f,x,y) in fixedFlights}:
w[x,y,f] = 1;
subject to fixToZero{(f,x,y) in fixedToZero}:
w[x,y,f] = 0;

subject to abs1{f in F}:
abs_t_ear[f] >= t_ear[f,e[f]] - t_hat_ear[f,e[f]];
subject to abs2{f in F}:
abs_t_ear[f] >= t_hat_ear[f,e[f]] - t_ear[f,e[f]];


#objective 49
#minimize z: sum{i in F,x in V, y in V: (x,y) in E} w[x,y,i];
#objective 50
minimize opt: sum{i in F} abs_t_ear[i];


/*
subject to trail1{i in F,j in F, x in V, y in V:(x,y) in E}:
(w[x,y,i]+ w[x,y,j]=2) ==> ((v[i,y]*(t[j,x]-t[i,x])>= D) or
(v[j,y]*(t[i,x]-t[j,x]) >=D));
subject to trail2{i in F,j in F, x in V, y in V:(x,y) in E}:
(w[x,y,i]+ w[x,y,j]=2) ==> ((v[i,y]*(t[j,y]-t[i,y])>= D) or (v[j,y]*(t[i,y]-t[j,y]) >=D));
subject to merge{i in F,j in F, x in V, x1 in V, x2 in V:(x1,x) in E and (x2,x) in E}:
(w[1,x,i]+ w[x2,x,j]=2) ==> ((t[j,x]-t[i,x]>=angleM[i,j,x]* D/v[j,x]) or (t[i,x]-t[j,x] >=
angleM[i,j,x]*D/v[i,x]));
subject to diver{i in F,j in F, x in V, x1 in V, x2 in V:(x,x1) in E and (x2,x) in E}:
(w[x,x1,i]+ w[x2,x,j]=2) ==> ((t[j,x]-t[i,x]>= anglePM[i,j,x]*(D/v[j,x]+ D/v[i,x1])) or
(t[i,x]-t[j,x] >= anglePM[i,j,x]*(D/v[i,x]+ D/v[j,x1])));
subject to split{i in F,j in F, x in V, x1 in V, x2 in V:(x,x1) in E and (x,x2) in E}:
(w[x,x1,i]+ w[x,x2,j]=2) ==> ((t[j,x]-t[i,x]>= angleP[i,j,x]* D/v[i,x1]) or (t[i,x]-t[j,x]
>=angleP[i,j,x]*D/v[j,x2]));
*/