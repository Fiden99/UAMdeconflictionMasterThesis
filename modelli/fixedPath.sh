#!/bin/bash
#SBATCH --job-name=singlePathHeuristic      # Nome del job
#SBATCH --output=/home/magi/UAMdeconflictionMasterThesis/modelli/out/fixed/ampl_output_%A_%a.txt  # File di output per stdout (%A = job ID, %a = array task ID)
#SBATCH --error=/home/magi/UAMdeconflictionMasterThesis/modelli/out/fixed/ampl_error_%A_%a.txt    # File di output per stderr
#SBATCH --ntasks=1                      # Numero di task per job
#SBATCH --cpus-per-task=1               # Numero di core per job
#SBATCH --array=0-30                    # Numero di job array

# Crea una lista dei nomi dei file, tempi, numero di nodi e voli dal file "fixedPathFiles.txt"
declare -A FILES_AND_TIMES
declare -A NODES_AND_FLIGHTS

while read -r line; do
    # Estrai i parametri dai campi del file
    nNodes=$(echo "$line" | cut -d ' ' -f 1)     # Numero di nodi (prima colonna)
    nFlights=$(echo "$line" | cut -d ' ' -f 2)   # Numero di voli (seconda colonna)
    name=$(echo "$line" | cut -d ' ' -f 3)       # Nome del file (terza colonna)
    time=$(echo "$line" | cut -d ' ' -f 4)       # Tempo (quarta colonna)
    
    # Aggiungi al dizionario
    FILES_AND_TIMES["$name"]=$time
    NODES_AND_FLIGHTS["$name"]="$nNodes $nFlights"
    
    # Debug: stampa per controllare i dati estratti
    echo "File letto: $name, Tempo: $time, Nodi: $nNodes, Voli: $nFlights"
done < /home/magi/UAMdeconflictionMasterThesis/modelli/fixedPathFiles.txt

# Trova i file .dat nella directory "data/" che corrispondono ai nomi in "fixedPathFiles.txt"
FILES=()
TIMES=()
NODES=()
FLIGHTS=()

for file in /home/magi/UAMdeconflictionMasterThesis/modelli/data/*.dat; do
    filename=$(basename "$file" .dat)
    if [[ -n "${FILES_AND_TIMES[$filename]}" ]]; then
        FILES+=("$file")
        TIMES+=("${FILES_AND_TIMES[$filename]}")
        IFS=' ' read -r node flight <<< "${NODES_AND_FLIGHTS[$filename]}"
        NODES+=("$node")
        FLIGHTS+=("$flight")
        
        # Debug: stampa il file trovato
        echo "File .dat trovato e associato: $filename"
    fi
done

# Definisci il job array in base al numero di file filtrati
NUM_FILES=${#FILES[@]}

# Verifica che l'indice dell'array non superi il numero di file disponibili
if [ $SLURM_ARRAY_TASK_ID -ge $NUM_FILES ]; then
    echo "Indice del job array ($SLURM_ARRAY_TASK_ID) superiore al numero di file disponibili ($NUM_FILES)."
    exit 1
fi

# Carica il modulo AMPL (se necessario, se AMPL è gestito tramite moduli)
# module load ampl

# Seleziona il file, il tempo, il numero di nodi e di voli corrispondente all'indice dell'array di Slurm
datFile=${FILES[$SLURM_ARRAY_TASK_ID]}
timeLimit=${TIMES[$SLURM_ARRAY_TASK_ID]}
nNodes=${NODES[$SLURM_ARRAY_TASK_ID]}
nFlights=${FLIGHTS[$SLURM_ARRAY_TASK_ID]}

# Ottieni il percorso assoluto del file corrente
datFileBase=$(basename "$datFile" .dat)

# Debug: stampa i parametri prima dell'esecuzione
echo "Esecuzione AMPL con i seguenti parametri:"
echo "File: $datFile"
echo "Tempo: $timeLimit"
echo "Nodi: $nNodes"
echo "Voli: $nFlights"
echo "File .dat base: $datFileBase"
echo "SLURM_ARRAY_TASK_ID: $SLURM_ARRAY_TASK_ID"

# Esegui AMPL con il file .dat corrente e i parametri specificati
timeLimit="$timeLimit" nNodes="$nNodes" nFlights="$nFlights" absPath=$PWD datFile=$datFileBase ampl /home/magi/UAMdeconflictionMasterThesis/modelli/fixedPath.run
