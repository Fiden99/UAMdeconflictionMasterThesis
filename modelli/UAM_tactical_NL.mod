#data
param nn; #number of nodes
set F; #set of flights
set AP within F default {};
set NC within F default {};
set V:= 0..nn-1; #set of nodes
set E within {V cross V}; #arcs
param s{F}; #starting point for each flight
param e{F}; #ending poitn for each flight
param d{E}>=0; #distance for each pair of nodes
param v_max;
param v_min; #param v{F,V}; #entering speed
param bigM:=1000; #bigM for linearizing purpose
param W:=1000; #used for AP
set conflictsNodes within {x in V, x1 in V, x2 in V: x1<>x2 and ((x,x1) in E or (x1,x) in E) and ((x,x2) in E or (x2,x) in E)};
param angle{conflictsNodes} ;
param D; # safety distance
param t_hat_ear{F,V} default 998;
param t_hat_lat{F,V} default 999;
param tini;
set fixedFlights within {F cross E};
set inFlight within {F};
param drifted_flight default -1;
param drifted_wp default -1;
#param v_climb;

#definition of sets and parameters not in .dat
#set centralConflictNode = setof{(x,x1,x2) in conflictsNodes} x;
set fixedF := setof {(f,x,y) in fixedFlights} f;
set freeF := F diff fixedF;
set fixedFNodes := {setof {(f,x,y) in fixedFlights} (f,x)} union {setof {(f,x,y) in fixedFlights} (f,y)};
param wFixed{(x,y) in E,f in fixedF} binary = if (f,x,y) in fixedFlights then 1 else 0;
set passedNode within fixedFNodes:= {(f,x) in fixedFNodes: f in NC or (f in inFlight and t_hat_ear[f,x] <= tini)};
set notPassedNode within fixedFNodes := fixedFNodes diff passedNode;
set firstNotPassed within notPassedNode:={(f,x) in notPassedNode: f in inFlight and 
    not exists {(j,r) in notPassedNode} (j == f and t_hat_ear[j,r] < t_hat_ear[f,x])};
set timeFixed := firstNotPassed union passedNode;

set passedFreeF within {freeF cross E} default {};
set passedFN := {setof {(f,x,y) in passedFreeF} (f,x)} union {setof {(f,x,y) in passedFreeF} (f,y)};
set passedFreeNode within passedFN:= {(f,x) in passedFN: f in inFlight and t_hat_ear[f,x] <= tini};
set notPassedFreeNode within passedFN := passedFN diff passedFreeNode;
set firstNotPassedFree within notPassedFreeNode:={(f,x) in notPassedFreeNode: f in inFlight and 
    not exists {(j,r) in notPassedFreeNode} (j == f and t_hat_ear[j,r] < t_hat_ear[f,x])};
set timeFixedFree = firstNotPassedFree union passedFreeNode;
#variables
var w{E,freeF} binary; #flight f pass through arc i,j
var t_down{(freeF cross V) diff timeFixedFree} >=0;
var t_up {(freeF cross V) diff timeFixedFree} >= 0;
var t_ear{freeF,V} >=0; 
var t_lat{freeF,V} >=0; 

var t_down_fixed{fixedFNodes diff timeFixed} >=0;
var t_up_fixed {fixedFNodes diff timeFixed} >= 0;
var t_ear_fixed{fixedFNodes} >=0; 
var t_lat_fixed{fixedFNodes} >=0; 

var t_ear_start{F} integer >=0;
var abs_t_ear{F diff (AP union NC)} >=0;

var delayPriority{AP} integer >= 0;

#binary variables for linearization of conflicts
var passFirst{i in freeF, j in freeF, x in V: i<>j} binary;
var pass2Fixeds{i in fixedF, j in fixedF, x in V: i<>j and (i,x) in fixedFNodes and (j,x) in fixedFNodes} binary;
var pass1Fixed{i in fixedF, j in freeF, x in V: i<>j and (i,x) in fixedFNodes} binary;
#constrains
subject to startInteger{f in freeF}:
t_ear[f,s[f]] = t_ear_start[f];

subject to startIntegerFixed{f in fixedF}:
t_ear_fixed[f,s[f]] = t_ear_start[f];

subject to afterprecalculated{f in F diff inFlight}:
tini <= t_ear_start[f];

subject to calculateLat{i in freeF, x in V}:
t_lat[i,x]=t_ear[i,x] + t_hat_lat[i,x] - t_hat_ear[i,x];

subject to calculateLatFixed{(i,x) in fixedFNodes diff timeFixed : i not in AP}:
t_lat_fixed[i,x]=t_ear_fixed[i,x] + t_hat_lat[i,x] - t_hat_ear[i,x];


subject to startingW{i in freeF}:
sum{(x,s[i]) in E} w[x,s[i],i] - sum{(s[i],x) in E} w[s[i],x,i]=-1;
subject to finishingW{i in freeF}:
sum{(x,e[i]) in E} w[x,e[i],i] - sum{(e[i],x) in E} w[e[i],x,i]=1;
subject to allW{i in freeF,x in V : x<> s[i] and x <> e[i]}:
sum{(x,y) in E} w[x,y,i]=sum{(y,x) in E} w[y,x,i];

subject to limitT_down{i in freeF, x in V : x <> s[i] and (i,x) not in timeFixedFree}:
t_down[i,x] <= t_ear[i,x];
subject to limitT_up{i in freeF , x in V: x <> s[i] and (i,x) not in timeFixedFree}:
t_ear[i,x] <= t_up[i,x];

#vedere se lo devo rimuovere anche da AP
subject to limitT_down_fixed{(i,x) in fixedFNodes diff timeFixed : x <> s[i]}:
t_down_fixed[i,x] <= t_ear_fixed[i,x];
subject to limitT_up_fixed{(i,x) in fixedFNodes diff timeFixed: x <> s[i]}:
t_ear_fixed[i,x] <= t_up_fixed[i,x];


subject to defineT_down{f in freeF ,y in V: y <> s[f] and (f,y) not in timeFixedFree}:
t_down[f,y]=sum{(x,y) in E} w[x,y,f] *(d[x,y]/v_max + t_ear[f,x]);
subject to defineT_up{f in freeF ,y in V: y <> s[f] and (f,y) not in timeFixedFree}:
t_up[f,y]=sum{(x,y) in E} w[x,y,f] * (d[x,y]/v_min + t_ear[f,x]);

subject to defineT_down_fixed{(f,x,y) in fixedFlights :(f,y) not in timeFixed and f not in (AP union NC)}:
t_down_fixed[f,y]=d[x,y]/v_max + t_ear_fixed[f,x];
subject to defineT_up_fixed{(f,x,y) in fixedFlights : (f,y) not in timeFixed and f not in (AP union NC)}:
t_up_fixed[f,y]=d[x,y]/v_min + t_ear_fixed[f,x];

#VALUTARE SE AGGIUNGERE I VINCOLI PER v_climb, in caso modificare fixed con (x != s[f] and y!= e[f]) nella definizione in fixed e (x!= s[f] and y!= e[f]) nella sommatoria in free
#subject to defineT_climb_down_fixed{(f,x,y) in fixedFlights : (x == s[f] or y==e[f]) and (f,y) not in timeFixed and f not in (AP union NC)}:
#t_down_fixed[f,y]=d[x,y]/v_max + t_ear_fixed[f,x];
#subject to defineT_climb_up_fixed{(f,x,y) in fixedFlights :(x == s[f] or y==e[f]) and  (f,y) not in timeFixed and f not in (AP union NC)}:
#t_up_fixed[f,y]=d[x,y]/v_climb + t_ear_fixed[f,x];

#subject to defineT_climb_down{f in freeF ,y in V: y <> s[f] and (f,y) not in timeFixedFree}:
#t_down[f,y]=sum{(x,y) in E: x == s[f] or y== e[f] } w[x,y,f] *(d[x,y]/v_climb + t_ear[f,x]);
#subject to defineT_climb_up{f in freeF ,y in V: y <> s[f] and (f,y) not in timeFixedFree}:
#t_up[f,y]=sum{(x,y) in E: x== s[f] or y== e[f]} w[x,y,f] * (d[x,y]/v_climb + t_ear[f,x]);

# conflicts

subject to trail13 {i in freeF,j in freeF, (x,y) in E: i<>j and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x,y,i] + w[x,y,j] == 2) ==> (t_ear[j,x]-t_lat[i,x] >= D/v_min* passFirst[i,j,x] -bigM* (1-passFirst[i,j,x]));
subject to trail14 {i in freeF,j in freeF, (x,y) in E: i<>j and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x,y,i] + w[x,y,j] == 2) ==> (t_ear[i,x]-t_lat[j,x]>=D/v_min* (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);

#subject to trail13Fixed {i in fixedF,j in fixedF, (x,y) in E: i<>j and (wFixed[x,y,i] + wFixed[x,y,j] == 2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
subject to trail13Fixed {i in fixedF,j in fixedF, (x,y) in E: i<>j and  (wFixed[x,y,i] + wFixed[x,y,j] == 2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[j,x]-t_lat_fixed[i,x] >= D/v_min * pass2Fixeds[i,j,x] - bigM* (1-pass2Fixeds[i,j,x]);
#subject to trail14Fixed {i in fixedF,j in fixedF, (x,y) in E: i<>j and (wFixed[x,y,i] + wFixed[x,y,j] == 2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
subject to trail14Fixed {i in fixedF,j in fixedF, (x,y) in E: i<>j and  (wFixed[x,y,i] + wFixed[x,y,j] == 2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[i,x]-t_lat_fixed[j,x]>=D/v_min* (1-pass2Fixeds[i,j,x]) - bigM*pass2Fixeds[i,j,x];

subject to trail13FixedI {i in fixedF,j in freeF, (x,y) in E: (wFixed[x,y,i]==1) and i<>j and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[j,x]-t_lat_fixed[i,x] >= D/v_min* pass1Fixed[i,j,x] - bigM*(1-pass1Fixed[i,j,x]) - bigM*(1-w[x,y,j]);
subject to trail14FixedI {i in fixedF,j in freeF, (x,y) in E: (wFixed[x,y,i]==1) and i<>j and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[i,x]-t_lat[j,x]>=D/v_min* (1-pass1Fixed[i,j,x]) - bigM*pass1Fixed[i,j,x] - bigM*(1-w[x,y,j]);

subject to trail13FixedJ {i in freeF,j in fixedF, (x,y) in E: (wFixed[x,y,j]==1) and i<>j and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[j,x]-t_lat[i,x] >= D/v_min* (1-pass1Fixed[j,i,x]) -bigM* (1-(1-pass1Fixed[j,i,x])) - bigM*(1-w[x,y,i]);
subject to trail14FixedJ {i in freeF,j in fixedF, (x,y) in E: (wFixed[x,y,j]==1) and i<>j and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[i,x]-t_lat_fixed[j,x]>=D/v_min* (1-(1-pass1Fixed[j,i,x])) - bigM*(1-pass1Fixed[j,i,x]) - bigM*(1-w[x,y,i]);


subject to trail23 {i in freeF,j in freeF, (x,y) in E: i<>j and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
(w[x,y,i] + w[x,y,j] == 2) ==> t_ear[j,y]-t_lat[i,y]>= D/v_min* passFirst[i,j,x] - bigM*(1-passFirst[i,j,x]);
subject to trail24 {i in freeF,j in freeF, (x,y) in E: i<>j and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
(w[x,y,i] + w[x,y,j] == 2) ==> t_ear[i,y]-t_lat[j,y]>= D/v_min * (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x];

#subject to trail23Fixed {i in fixedF,j in fixedF, (x,y) in E: i<>j and (wFixed[x,y,i] + wFixed[x,y,j] == 2) and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
subject to trail23Fixed {i in fixedF,j in fixedF, (x,y) in E: i<>j and (wFixed[x,y,i] + wFixed[x,y,j] == 2) and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
t_ear_fixed[j,y]-t_lat_fixed[i,y]>= D/v_min* pass2Fixeds[i,j,x] - bigM*(1-pass2Fixeds[i,j,x]);
#subject to trail24Fixed {i in fixedF,j in fixedF, (x,y) in E: i<>j and (wFixed[x,y,i] + wFixed[x,y,j] == 2) and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
subject to trail24Fixed {i in fixedF,j in fixedF, (x,y) in E: i<>j and (wFixed[x,y,i] + wFixed[x,y,j] == 2) and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
t_ear_fixed[i,y]-t_lat_fixed[j,y]>= D/v_min * (1-pass2Fixeds[i,j,x]) - bigM*pass2Fixeds[i,j,x];

subject to trail23FixedI {i in fixedF,j in freeF, (x,y) in E: i<>j and (wFixed[x,y,i]==1) and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
t_ear[j,y]-t_lat_fixed[i,y]>= D/v_min* pass1Fixed[i,j,x] - bigM*(1-pass1Fixed[i,j,x]) - bigM*(1-w[x,y,j]);
subject to trail24FixedI {i in fixedF,j in freeF, (x,y) in E: i<>j and (wFixed[x,y,i]==1) and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
t_ear_fixed[i,y]-t_lat[j,y]>= D/v_min * (1-pass1Fixed[i,j,x]) - bigM*pass1Fixed[i,j,x] - bigM*(1-w[x,y,j]);

subject to trail23FixedJ {i in freeF,j in fixedF, (x,y) in E: i<>j and (wFixed[x,y,j]==1) and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
t_ear_fixed[j,y]-t_lat[i,y]>= D/v_min* (1-pass1Fixed[j,i,x]) - bigM*(1-(1-pass1Fixed[j,i,x])) - bigM*(1-w[x,y,i]);
subject to trail24FixedJ {i in freeF,j in fixedF, (x,y) in E: i<>j and (wFixed[x,y,j]==1) and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
t_ear[i,y]-t_lat_fixed[j,y]>= D/v_min * (1-(1-pass1Fixed[j,i,x])) - bigM*(1-pass1Fixed[j,i,x]) - bigM*(1-w[x,y,i]);


subject to merge3 {i in freeF, j in freeF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x1,x,i]+w[x2,x,j] == 2) ==> (t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min * passFirst[i,j,x] - bigM*(1-passFirst[i,j,x]));
subject to merge3_1 {i in freeF, j in freeF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x1,x,j]+w[x2,x,i] == 2) ==> (t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min * passFirst[i,j,x] - bigM*(1-passFirst[i,j,x]));

#subject to merge3Fixed {i in fixedF, j in fixedF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and (wFixed[x1,x,i]+wFixed[x2,x,j] == 2) and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
subject to merge3Fixed {i in fixedF, j in fixedF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and  i<>j and (wFixed[x1,x,i]+wFixed[x2,x,j] == 2) and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*D/v_min * pass2Fixeds[i,j,x] - bigM*(1-pass2Fixeds[i,j,x]);
#subject to merge3_1Fixed {i in fixedF, j in fixedF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x1,x,j]+wFixed[x2,x,i] == 2) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
subject to merge3_1Fixed {i in fixedF, j in fixedF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and  (wFixed[x1,x,j]+wFixed[x2,x,i] == 2) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*D/v_min * pass2Fixeds[i,j,x] - bigM*(1-pass2Fixeds[i,j,x]);

subject to merge3FixedI {i in fixedF, j in freeF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x1,x,i]==1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*D/v_min * pass1Fixed[i,j,x] - bigM*(1-pass1Fixed[i,j,x]) - bigM*(1-w[x2,x,j]);
subject to merge3_1FixedI {i in fixedF, j in freeF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x2,x,i]==1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*D/v_min * pass1Fixed[i,j,x] - bigM*(1-pass1Fixed[i,j,x]) - bigM*(1-w[x1,x,j]);

subject to merge3FixedJ {i in freeF, j in fixedF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x2,x,j] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min * (1-pass1Fixed[j,i,x]) - bigM*(1-(1-pass1Fixed[j,i,x])) - bigM*(1-w[x1,x,i]);
subject to merge3_1FixedJ {i in freeF, j in fixedF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x1,x,j] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min * (1-pass1Fixed[j,i,x]) - bigM*(1-(1-pass1Fixed[j,i,x])) - bigM*(1-w[x2,x,i]);



subject to merge4 {i in freeF, j in freeF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x1,x,j]+w[x2,x,i] == 2) ==> (t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min* (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);
subject to merge4_1 {i in freeF, j in freeF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x1,x,i]+w[x2,x,j] == 2) ==> (t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min* (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);

#subject to merge4Fixed {i in fixedF, j in fixedF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x1,x,j]+wFixed[x2,x,i] == 2) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
subject to merge4Fixed {i in fixedF, j in fixedF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and  (wFixed[x1,x,j]+wFixed[x2,x,i] == 2) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*D/v_min* (1-pass2Fixeds[i,j,x]) - bigM*pass2Fixeds[i,j,x];
#subject to merge4_1Fixed {i in fixedF, j in fixedF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x1,x,i]+wFixed[x2,x,j] == 2) and i<>j  and  x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
subject to merge4_1Fixed {i in fixedF, j in fixedF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and  (wFixed[x1,x,i]+wFixed[x2,x,j] == 2) and i<>j  and  x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*D/v_min* (1-pass2Fixeds[i,j,x]) - bigM*pass2Fixeds[i,j,x];

subject to merge4FixedI {i in fixedF, j in freeF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x2,x,i]==1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min* (1-pass1Fixed[i,j,x]) - bigM*pass1Fixed[i,j,x] - bigM*(1-w[x1,x,j]);
subject to merge4_1FixedI {i in fixedF, j in freeF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x1,x,i]==1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min* (1-pass1Fixed[i,j,x]) - bigM*pass1Fixed[i,j,x] - bigM*(1-w[x2,x,j]);

subject to merge4FixedJ {i in freeF, j in fixedF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x1,x,j] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*D/v_min* (1-(1-pass1Fixed[j,i,x])) - bigM*(1-pass1Fixed[j,i,x]) - bigM*(1-w[x2,x,i]);
subject to merge4_1FixedJ {i in freeF, j in fixedF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x2,x,j]==1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*D/v_min* (1-(1-pass1Fixed[j,i,x])) - bigM*(1-pass1Fixed[j,i,x]) -bigM*(1-w[x1,x,i]);


#valutare eliminazione di x<>y
subject to diver3 {i in freeF, j in freeF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
(w[x,x1,i]+w[x2,x,j] == 2) ==> (t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)*passFirst[i,j,x] - bigM*(1-passFirst[i,j,x])) ;
subject to diver3_1 {i in freeF, j in freeF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
(w[x,x1,j]+w[x2,x,i] == 2) ==> (t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)*passFirst[i,j,x] - bigM*(1-passFirst[i,j,x])) ;

#subject to diver3Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,i]+wFixed[x2,x,j] == 2) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
subject to diver3Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and  (wFixed[x,x1,i]+wFixed[x2,x,j] == 2) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
t_ear_fixed[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)*pass2Fixeds[i,j,x] - bigM*(1-pass2Fixeds[i,j,x]);
#subject to diver3_1Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,j]+wFixed[x2,x,i] == 2) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
subject to diver3_1Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and  (wFixed[x,x1,j]+wFixed[x2,x,i] == 2) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
t_ear_fixed[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)*pass2Fixeds[i,j,x] - bigM*(1-pass2Fixeds[i,j,x]) ;

subject to diver3FixedI {i in fixedF, j in freeF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,i]==1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
t_ear[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)*pass1Fixed[i,j,x] - bigM*(1-pass1Fixed[i,j,x]) - bigM*(1-w[x2,x,j]) ;
subject to diver3_1FixedI {i in fixedF, j in freeF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x2,x,i]==1) and  i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
t_ear[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)*pass1Fixed[i,j,x] - bigM*(1-pass1Fixed[i,j,x]) - bigM*(1-w[x,x1,j]) ;

subject to diver3FixedJ {i in freeF, j in fixedF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x2,x,j]==1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
t_ear_fixed[j,x]- t_lat[i,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)*(1-pass1Fixed[j,i,x]) - bigM*(1-(1-pass1Fixed[j,i,x])) - bigM*(1-w[x,x1,i]) ;
subject to diver3_1FixedJ {i in freeF, j in fixedF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,j]==1)and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
t_ear_fixed[j,x]- t_lat[i,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)*(1-pass1Fixed[j,i,x]) - bigM*(1-(1-pass1Fixed[j,i,x])) - bigM*(1-w[x2,x,i]) ;


subject to diver4 {i in freeF, j in freeF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes  and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
(w[x,x1,i]+w[x2,x,j] == 2) ==> (t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)* (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);
subject to diver4_1 {i in freeF, j in freeF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
(w[x,x1,j]+w[x2,x,i] == 2) ==> (t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)* (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);

#subject to diver4Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and (wFixed[x,x1,i]+wFixed[x2,x,j] == 2) and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
subject to diver4Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and  i<>j and (wFixed[x,x1,i]+wFixed[x2,x,j] == 2) and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
t_ear_fixed[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)* (1-pass2Fixeds[i,j,x]) - bigM*pass2Fixeds[i,j,x];
#subject to diver4_1Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and (wFixed[x,x1,j]+wFixed[x2,x,i] == 2) and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
subject to diver4_1Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and  i<>j and (wFixed[x,x1,j]+wFixed[x2,x,i] == 2) and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
t_ear_fixed[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)* (1-pass2Fixeds[i,j,x]) - bigM*pass2Fixeds[i,j,x];

subject to diver4FixedI {i in fixedF, j in freeF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,i]==1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
t_ear_fixed[i,x]- t_lat[j,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)* (1-pass1Fixed[i,j,x]) - bigM*pass1Fixed[i,j,x] - bigM*(1-w[x2,x,j]);
subject to diver4_1FixedI {i in fixedF, j in freeF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x2,x,i]==1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
t_ear_fixed[i,x]- t_lat[j,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)* (1-pass1Fixed[i,j,x]) - bigM*pass1Fixed[i,j,x] - bigM*(1-w[x,x1,j]);

subject to diver4FixedJ {i in freeF, j in fixedF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x2,x,j] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
t_ear[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)* (1-(1-pass1Fixed[j,i,x])) - bigM*(1-pass1Fixed[j,i,x]) - bigM*(1-w[x,x1,i]);
subject to diver4_1FixedJ {i in freeF, j in fixedF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,j] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
t_ear[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)* (1-(1-pass1Fixed[j,i,x])) - bigM*(1-pass1Fixed[j,i,x]) - bigM*(1-w[x2,x,i]);



subject to split3 {i in freeF, j in freeF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x,x1,i]+w[x,x2,j] == 2) ==> (t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min*passFirst[i,j,x] - bigM*(1-passFirst[i,j,x]));
subject to split3_1 {i in freeF, j in freeF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x,x1,j]+w[x,x2,i] == 2) ==> (t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min*passFirst[i,j,x] - bigM*(1-passFirst[i,j,x]));

#subject to split3Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,i]+wFixed[x,x2,j] == 2) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
subject to split3Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and  (wFixed[x,x1,i]+wFixed[x,x2,j] == 2) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*D/v_min*pass2Fixeds[i,j,x] - bigM*(1-pass2Fixeds[i,j,x]);
#subject to split3_1Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,j]+wFixed[x,x2,i] == 2) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
subject to split3_1Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and  (wFixed[x,x1,j]+wFixed[x,x2,i] == 2) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*D/v_min*pass2Fixeds[i,j,x] - bigM*(1-pass2Fixeds[i,j,x]);

subject to split3FixedI {i in fixedF, j in freeF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,i] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*D/v_min*pass1Fixed[i,j,x] - bigM*(1-pass1Fixed[i,j,x]) - bigM*(1-w[x,x2,j]);
subject to split3_1FixedI {i in fixedF, j in freeF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x2,i] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*D/v_min*pass1Fixed[i,j,x] - bigM*(1-pass1Fixed[i,j,x]) - bigM*(1-w[x,x1,j]);

subject to split3FixedJ {i in freeF, j in fixedF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x2,j]==1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min*(1-pass1Fixed[j,i,x]) - bigM*(1-(1-pass1Fixed[j,i,x])) - bigM*(1-w[x,x1,i]);
subject to split3_1FixedJ {i in freeF, j in fixedF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,j] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min*(1-pass1Fixed[j,i,x]) - bigM*(1-(1-pass1Fixed[j,i,x])) - bigM*(1-w[x,x2,i]);



subject to split4 {i in freeF, j in freeF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x,x1,i]+w[x,x2,j] == 2) ==> (t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min * (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);
subject to split4_1 {i in freeF, j in freeF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x,x1,j]+w[x,x2,i] == 2) ==> (t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min * (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);

#subject to split4Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and i<>j and (wFixed[x,x1,i]+wFixed[x,x2,j] == 2) and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
subject to split4Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and  i<>j and (wFixed[x,x1,i]+wFixed[x,x2,j] == 2) and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*D/v_min * (1-pass2Fixeds[i,j,x]) - bigM*pass2Fixeds[i,j,x];
#subject to split4_1Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and i<>j and (wFixed[x,x1,j]+wFixed[x,x2,i] == 2) and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
subject to split4_1Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and  i<>j and (wFixed[x,x1,j]+wFixed[x,x2,i] == 2) and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*D/v_min * (1-pass2Fixeds[i,j,x]) - bigM*pass2Fixeds[i,j,x];

subject to split4FixedI {i in fixedF, j in freeF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,i] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min * (1-pass1Fixed[i,j,x]) - bigM*pass1Fixed[i,j,x] - bigM*(1-w[x,x2,j]);
subject to split4_1FixedI {i in fixedF, j in freeF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x2,i] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min * (1-pass1Fixed[i,j,x]) - bigM*pass1Fixed[i,j,x] - bigM*(1-w[x,x1,j]);

subject to split4FixedJ {i in freeF, j in fixedF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x2,j] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*D/v_min * (1-(1-pass1Fixed[j,i,x])) - bigM*(1-pass1Fixed[j,i,x]) - bigM*(1-w[x,x1,i]);
subject to split4_1FixedJ {i in freeF, j in fixedF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,j] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*D/v_min * (1-(1-pass1Fixed[j,i,x])) - bigM*(1-pass1Fixed[j,i,x]) - bigM*(1-w[x,x2,i]);

/*
subject to driftEar:
t_ear[drifted_flight,drifted_wp] >= drifted_t_ear_fix;
#TODO controllare se devo inserire t_ear o t_lat
subject to driftLat:
t_lat[drifted_flight,drifted_wp] <= drifted_t_lat_fix;
subject to driftWP:
sum{(x,drifted_wp) in E} w[x,drifted_wp,drifted_flight] = 1;
*/

subject to abs1{f in freeF}:
abs_t_ear[f] >= t_ear[f,e[f]] - t_hat_ear[f,e[f]];
subject to abs2{f in freeF}:
abs_t_ear[f] >= t_hat_ear[f,e[f]] - t_ear[f,e[f]];


subject to abs1Fixed{f in fixedF diff (AP union NC)}:
abs_t_ear[f] >= t_ear_fixed[f,e[f]] - t_hat_ear[f,e[f]];
subject to abs2Fixed{f in fixedF diff (AP union NC)}:
abs_t_ear[f] >= t_hat_ear[f,e[f]] - t_ear_fixed[f,e[f]];


subject to fixPassedEar{(f,x) in timeFixed diff {(drifted_flight,drifted_wp)}}:
t_ear_fixed[f,x] = t_hat_ear[f,x];
subject to fixPassedLat{(f,x) in timeFixed diff {(drifted_flight,drifted_wp)}}:
t_lat_fixed[f,x] = t_hat_lat[f,x];


subject to fixFreeEar{(f,x) in timeFixedFree}:
t_ear[f,x] = t_hat_ear[f,x];
subject to fixFreeLat{(f,x) in timeFixedFree}:
t_lat[f,x] = t_hat_lat[f,x];
#subject to fixFreePath{(f,x) in timeFixedFree, y in V : (f,x,y) in passedFreeF}:
subject to fixFreePath{(f,y) in timeFixedFree, x in V : (f,x,y) in passedFreeF}:
w[x,y,f] = 1;

subject to priority1{(f,x) in fixedFNodes : f in AP}:
t_ear_fixed[f,x] = t_hat_ear[f,x] + delayPriority[f];
subject to priority2{(f,x) in fixedFNodes : f in AP}:
t_lat_fixed[f,x] = t_hat_lat[f,x] + delayPriority[f];


#capire se tenere o meno
subject to fixpass2Fixeds{i in fixedF, j in fixedF, x in V: i<>j and (i,x) in fixedFNodes and (j,x) in fixedFNodes}:
pass2Fixeds[i,j,x] + pass2Fixeds[j,i,x] <= 1;

set unavailableEdges = setof{(x,y,f) in fixedFlights: f in NC} (x,y);
subject to avoidShortcut {f in freeF, (x,y) in unavailableEdges}:
w[x,y,f] = 0;


minimize opt: sum{i in AP} W * delayPriority[i] + sum{i in F diff (AP union NC)} abs_t_ear[i];

/*
capire se mi serve

set TRAIL_CONF_1:={(i,j,m,h) in TRAIL_CONF: i not in NOT_ORDERED_TRIPS and j not in NOT_ORDERED_TRIPS};
set TRAIL_LAND_1:={(i,j,m,h) in TRAIL_LAND: i not in NOT_ORDERED_TRIPS and j not in NOT_ORDERED_TRIPS};
set TRAIL_TAKEOFF_1:={(i,j,m,h) in TRAIL_TAKEOFF: i not in NOT_ORDERED_TRIPS and j not in NOT_ORDERED_TRIPS};
set TRAIL_CRUISE_1:={(i,j,m,h) in TRAIL_CRUISE: i not in NOT_ORDERED_TRIPS and j not in NOT_ORDERED_TRIPS};
set CROSS_CONF_1:={(i,j,m,h_i,l_i,h_j,l_j) in CROSS_CONF: i not in NOT_ORDERED_TRIPS and j not in NOT_ORDERED_TRIPS};
set MERGE_CONF_1:={(i,j,m,h_i,h_j,l) in MERGE_CONF: i not in NOT_ORDERED_TRIPS and j not in NOT_ORDERED_TRIPS};
set SPLIT_CONF_1:={(i,j,m,h,l_i,l_j) in SPLIT_CONF: i not in NOT_ORDERED_TRIPS and j not in NOT_ORDERED_TRIPS};
set MERGE_LAND_1:={(i,j,m,h_i,h_j) in MERGE_LAND: i not in NOT_ORDERED_TRIPS and j not in NOT_ORDERED_TRIPS};
set SPLIT_TAKEOFF_1:={(i,j,m,l_i,l_j) in SPLIT_TAKEOFF: i not in NOT_ORDERED_TRIPS and j not in NOT_ORDERED_TRIPS};



# 4. Recommended travel times define intervals [tmin,tmax] and keep scheduled passing order
s.t. timeMargin{(i,m) in NOTFIXED_NODESPATH: i not in AP}:  tmax[i,m]=tmin[i,m]+tplanMax[i,m]-tplanMin[i,m];
s.t. passOrder_trail{(i,j,m,h) in TRAIL_CONF_1}: tmax[i,m] <= tmin[j,m];
s.t. passOrder_cross{(i,j,m,h_i,l_i,h_j,l_j) in CROSS_CONF_1}: tmax[i,m] <= tmin[j,m];
s.t. passOrder_merge{(i,j,m,h_i,h_j,l) in MERGE_CONF_1}: tmax[i,m]<= tmin[j,m];
s.t. passOrder_split{(i,j,m,h_i,h_j,l) in SPLIT_CONF_1}: tmax[i,m] <= tmin[j,m];

capire se l'abbiamo di già con i vincoli "Fixed"

s.t. sepCross_A{(i,j,m,h_i,l_i,h_j,l_j) in CROSS_CONF_1}: 
		tmin[j,m]-tmax[i,m] >= (D/vmin)*S_angle[m,h_i,h_j]; #tmin[j,m]-tmax[i,m] >= (D/vmin[j,m])*S_angle[m,h_i,h_j];
s.t. sepMerge{(i,j,m,h_i,h_j,l) in MERGE_CONF_1}: 
		tmin[j,m]-tmax[i,m] >= (D/vmin)*S_angle[m,h_i,h_j]; #tmin[j,m]-tmax[i,m] >= (D/vmin[j,m])*S_angle[m,h_i,h_j];
# --Type B: leg i=2, leg j=1 (j approaching m, i leaving m)
s.t. sepCross_B{(i,j,m,h_i,l_i,h_j,l_j) in CROSS_CONF_1}: 
		tmin[j,m]-tmax[i,m] >= (2*D/vmin) * S_angle[m,h_j,l_i]; #tmin[j,m]-tmax[i,m] >= (D/vmin[i,l_i]+D/vmin[j,m]) * S_angle[m,h_j,l_i];
# --Type C: leg i=2, leg j=2 (paths diverging at m)						  
s.t. sepCross_C{(i,j,m,h_i,l_i,h_j,l_j) in CROSS_CONF_1}: 
		tmin[j,m]-tmax[i,m] >= (D/vmin) * S_angle[m,l_i,l_j]; #tmin[j,m]-tmax[i,m] >= (D/vmin[i,l_i]) * S_angle[m,l_i,l_j];
s.t. sepSplit{(i,j,m,h,l_i,l_j) in SPLIT_CONF_1}: 
		tmin[j,m]-tmax[i,m] >= (D/vmin) * S_angle[m,l_i,l_j]; #tmin[j,m]-tmax[i,m] >= (D/vmin[i,l_i]) * S_angle[m,l_i,l_j];		

*/