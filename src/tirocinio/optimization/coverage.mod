#### DATI

param coeff_coverage;
param coeff_distanze;
param coeff_costo_guardie;
param min_coverage;
param nW;
param nG;
set Witnesses := 1..nW;
set Guardie := 1..nG;
set Costi_Guardie  := 1..nG;
set Copertura within Witnesses cross Guardie;
set Distanze within Guardie cross Guardie;

#### VARIABILI

var z;
var scelta_guardie {Guardie} binary;
var coperto {Witnesses};

minimize Total_Cost: 
    z;

#### VINCOLI
s.t. Costo_assegnamento:
    z = coeff_coverage * (sum {g in Guardie} scelta_guardie[g]) + coeff_distanze * (sum {g1 in Guardie, g2 in Guardie} scelta_guardie[g1] * scelta_guardie[g2] * distanze[g1,g2]);

s.t. Copertura {w in Witnesses}:
    coperto[w] = sum {g in Guardie} Copertura[w,g];

s.t. Copertura_minima:
    min_coverage <= sum {w in Witnesses} coperto / nG;