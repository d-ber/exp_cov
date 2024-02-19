#### DATI

param coeff_coverage; # peso copertura nell'obiettivo
param coeff_distanze; # peso distanze tra guardie nell'obiettivo
param min_coverage; # frazione (tra 0 e 1) di copertura minima richiesta
param nW; # numero witnesses (punti poligono)
param nG; # numero guardie (posizioni del robot)
set Witnesses := 1..nW; # insieme witnesses
set Guardie := 1..nG; # insieme guardie 
param costi_guardie {Guardie}; # costo scelta guardia (rischio, ...)
param copertura {Witnesses, Guardie}; # copertura[w, g] = 1 se la guardia g copre il witness w, 0 altrimenti
param distanza {Guardie, Guardie}; # distanze[g1, g2] = distanza tra le guardie g1 e g2

#### VARIABILI

var z; # var obiettivo, il costo totale
var scelta_guardie {Guardie} binary; # scelta_guardie[g] = 1 se scelgo la guardia g, 0 altrimenti
var scelta_guardie_coppie {Guardie, Guardie} binary; # variabile ausiliaria per linearizzare il prodotto "scelta_guardie[g1] * scelta_guardie[g2]" (vedi https://or.stackexchange.com/questions/37/how-to-linearize-the-product-of-two-binary-variables)
var coperto {Witnesses} binary; # coperto[w] = 1 se copro il testimone w, 0 altrimenti

# Voglio minimizzare il costo totale
minimize Total_Cost: 
    z;

#### VINCOLI
# Definisco il costo totale come peso_coverage*coverage + peso_distanze*distanze
s.t. Costo_assegnamento:
    z = coeff_coverage * (sum {g in Guardie} costi_guardie[g] * scelta_guardie[g]) + coeff_distanze * (sum {g1 in Guardie, g2 in Guardie} scelta_guardie_coppie[g1, g2] * distanza[g1, g2]);

# Vincoli per linearizzare "scelta_guardie[g1] * scelta_guardie[g2]" in una var binaria semplice e per non rendere la fun. ob. quadratica
s.t. Linearizzazione1 {g1 in Guardie, g2 in Guardie}:
    scelta_guardie_coppie[g1, g2] >= scelta_guardie[g1] + scelta_guardie[g2] - 1;

s.t. Linearizzazione2 {g1 in Guardie, g2 in Guardie}:
    scelta_guardie_coppie[g1, g2] <= scelta_guardie[g1];

s.t. Linearizzazione3 {g1 in Guardie, g2 in Guardie}:
    scelta_guardie_coppie[g1, g2] <= scelta_guardie[g2];

# Ogni witness deve essere coperto (avere una guardia che lo copre scelto), a meno che non sia disabilitato, ovvero fuori dalla copertura (vedi vincolo Copertura_minima)
s.t. Copertura_witnesses {w in Witnesses}:
    sum {g in Guardie: copertura[w, g]} scelta_guardie[g]  >= 1 * coperto[w];

# Imposto di coprire almeno una certa percentuale di witnesses
s.t. Copertura_minima:
    (sum {w in Witnesses} coperto[w]) / nW >= min_coverage;