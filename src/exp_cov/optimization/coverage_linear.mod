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
var guard_couple_choice {Guards, Guards} binary; # auxiliary var to linearize guard_choice[g1]*guard_choice[g2]

# Minimize total cost
minimize Total_Cost: 
    z;

#### CONSTRAINTS

# Define total cost as coverage_coeff*coverage + distance_coeff*distance
s.t. Choice_Cost:
    z = coverage_coeff * (sum {g in Guards} costi_Guards[g] * guard_choice[g]) + distance_coeff * (sum {g1 in Guards, g2 in Guards} guard_choice[g1] * guard_choice[g2] * distance[g1, g2]);

# Each witness must be covered by at least one guard, unless it is disabled
s.t. Coverage_Witnesses {w in Witnesses}:
    sum {g in Guards: coverage[w, g]} guard_choice[g]  >= 1 * covered[w];

# At least min_coverage*nW witnesses must be covered
s.t. Minimum_Coverage:
    (sum {w in Witnesses} covered[w]) / nW >= min_coverage;

# To linearize guard_choice[g1]*guard_choice[g2]
s.t. Linearization1 {g1 in Guards, g2 in Guards}:
    guard_couple_choice[g1, g2] >= guard_choice[g1] + guard_choice[g2] - 1;

s.t. Linearization2 {g1 in Guards, g2 in Guards}:
    guard_couple_choice[g1, g2] <= guard_choice[g1];

s.t. Linearization3 {g1 in Guards, g2 in Guards}:
    guard_couple_choice[g1, g2] <= guard_choice[g2];