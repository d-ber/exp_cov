#### DATA

param coverage_coeff; # coverage weight in objective function
param distance_coeff; # guard distance weight in objective function
param min_coverage; # minimum required coverage function, 0 <= min_coverage <= 1
param nW; # witness number (polygon points)
param nG; # guard number (robot poses)
set Witnesses := 1..nW; # witness set
set Guards := 1..nG; # guard set
param guard_cost {Guards}; # guard choice cost
param coverage {Witnesses, Guards}; # coverage[w, g] = 1 if guard g covers witness w, else 0
param distance {Guards, Guards}; # distance[g1, g2] = distance between g1 and g2


#### VARIABLES

var z; # objective var, total cost
var guard_choice {Guards} binary; # guard_choice[g] = 1 if g is chosen, else 0
var covered {Witnesses} binary; # covered[w] = 1 if w is covered, else 0
var arc_choice {Guards, Guards} binary; # arc_choice[g1, g2] = 1 if (g1, g2) arc is in the MST
var root {Guards} binary; # root[g] = 1 if g is the MST root (used for flux)
var flux {Guards, Guards} >= 0; # flux from g1 to g2
var vertex_n >= 0; # number of vertexes in MST

# Minimize total cost
minimize Total_Cost: 
    z;

#### CONSTRAINTS

# Define total cost as coeff_coverage*coverage + coeff_distanze*distance
s.t. Choice_Cost:
    z = coverage_coeff * (sum {g in Guards} costi_Guards[g] * guard_choice[g]) + distance_coeff * (sum {g1 in Guards, g2 in Guards: g1<>g2} arc_choice[g1, g2] * distance[g1, g2]);

# Each witness must be covered by at least one guard, unless it is disabled
s.t. Coverage_Witnesses {w in Witnesses}:
    sum {g in Guards: coverage[w, g]} guard_choice[g]  >= 1 * covered[w];

# At least min_coverage*nW witnesses must be covered
s.t. Minimum_Coverage:
    (sum {w in Witnesses} covered[w]) / nW >= min_coverage;

## Vincoli Minimum Spanning Tree (MST)
# Definizione var numero vertici
s.t. Numero_vertici:
    vertex_n = sum {g in Guards} scelta_Guards[g];

# Posso scegliere archi solo tra coppie di Guards scelte
s.t. Archi_su_scelta {g1 in Guards, g2 in Guards: g1<>g2}:
    arc_choice[g1, g2] + arc_choice[g2, g1]  <= scelta_Guards[g1] * scelta_Guards[g2];

# Scelgo una sola root
s.t. root_singola1:
    sum {r in Guards} root[r] = 1;

# Scelgo la root tra le Guards scelte
s.t. root_singola2:
    sum {r in Guards} root[r] * scelta_Guards[r] = 1;

# Vincoli di avere come numero di archi scelti vertex_n - 1
s.t. Totale_archi:
    sum {g1 in Guards, g2 in Guards: g1<>g2} arc_choice[g1, g2] = vertex_n - 1;

# Il flux dalla root è n-1
s.t. root_flux:
    (sum {r in Guards, g2 in Guards: g2<>r} flux[r, g2] * root[r]) = vertex_n - 1;

# Ogni vertice non root consuma flux 1
s.t. Consumo_flux {g in Guards}:
    (1-root[g]) * scelta_Guards[g] * ((sum {g1 in Guards: g<>g1} flux[g1, g]) - (sum {g2 in Guards: g<>g2} flux[g, g2])) = (1-root[g]) *  scelta_Guards[g];

# flux possibile solo se arco in albero, e limite superiore di vertex_n - 1
s.t. flux_in_albero {g1 in Guards, g2 in Guards: g1<>g2}:
    flux[g1, g2] <= (vertex_n - 1) * arc_choice[g1, g2];