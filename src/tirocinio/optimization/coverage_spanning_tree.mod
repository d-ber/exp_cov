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
var coperto {Witnesses} binary; # coperto[w] = 1 se copro il testimone w, 0 altrimenti
var scelta_arco {Guardie, Guardie} binary; # scelta_arco[g1, g2] = 1 se scelgo di connettere g1 e g2 nel mio albero spanning tree minimo
var radice {Guardie} binary; # radice[g] = 1 se g è la radice dell'albero (per il flusso)
var flusso {Guardie, Guardie} >= 0; # flusso da g1 a g2
var n_vertici >= 0; # numero vertici in albero

# Voglio minimizzare il costo totale
minimize Total_Cost: 
    z;

#### VINCOLI
# Definisco il costo totale come peso_coverage*coverage + peso_distanze*distanze
s.t. Costo_assegnamento:
    z = coeff_coverage * (sum {g in Guardie} costi_guardie[g] * scelta_guardie[g]) + coeff_distanze * (sum {g1 in Guardie, g2 in Guardie: g1<>g2} scelta_arco[g1, g2] * distanza[g1, g2]);

# Ogni witness deve essere coperto (avere una guardia che lo copre scelto), a meno che non sia disabilitato, ovvero fuori dalla copertura (vedi vincolo Copertura_minima)
s.t. Copertura_witnesses {w in Witnesses}:
    sum {g in Guardie: copertura[w, g]} scelta_guardie[g]  >= 1 * coperto[w];

# Imposto di coprire almeno una certa percentuale di witnesses
s.t. Copertura_minima:
    (sum {w in Witnesses} coperto[w]) / nW >= min_coverage;

## Vincoli Minimum Spanning Tree (MST)
# Definizione var numero vertici
s.t. Numero_vertici:
    n_vertici = sum {g in Guardie} scelta_guardie[g];

# Posso scegliere archi solo tra coppie di guardie scelte
s.t. Archi_su_scelta {g1 in Guardie, g2 in Guardie: g1<>g2}:
    scelta_arco[g1, g2] + scelta_arco[g2, g1]  <= scelta_guardie[g1] * scelta_guardie[g2];

# Scelgo una sola radice
s.t. Radice_singola1:
    sum {r in Guardie} radice[r] = 1;

# Scelgo la radice tra le guardie scelte
s.t. Radice_singola2:
    sum {r in Guardie} radice[r] * scelta_guardie[r] = 1;

# Vincoli di avere come numero di archi scelti n_vertici - 1
s.t. Totale_archi:
    sum {g1 in Guardie, g2 in Guardie: g1<>g2} scelta_arco[g1, g2] = n_vertici - 1;

# Il flusso dalla radice è n-1
s.t. Radice_flusso:
    (sum {r in Guardie, g2 in Guardie: g2<>r} flusso[r, g2] * radice[r]) = n_vertici - 1;

# Ogni vertice non radice consuma flusso 1
s.t. Consumo_flusso {g in Guardie}:
    (1-radice[g]) * scelta_guardie[g] * ((sum {g1 in Guardie: g<>g1} flusso[g1, g]) - (sum {g2 in Guardie: g<>g2} flusso[g, g2])) = (1-radice[g]) *  scelta_guardie[g];

# Flusso possibile solo se arco in albero, e limite superiore di n_vertici - 1
s.t. Flusso_in_albero {g1 in Guardie, g2 in Guardie: g1<>g2}:
    flusso[g1, g2] <= (n_vertici - 1) * scelta_arco[g1, g2];