#!/bin/bash
#SBATCH --job-name=singlePathHeuristic      # Nome del job
#SBATCH --output=/home/magi/UAMdeconflictionMasterThesis/modelli/out/ampl_output_%A_%a.txt  # File di output per stdout (%A = job ID, %a = array task ID)
#SBATCH --error=/home/magi/UAMdeconflictionMasterThesis/modelli/out/ampl_error_%A_%a.txt    # File di output per stderr
#SBATCH --ntasks=1                      # Numero di task per job
#SBATCH --cpus-per-task=1               # Numero di core per job
#SBATCH --array=0-59

# Crea una lista dei nomi dei file e dei tempi dal file "moreIterations"
declare -A FILES_AND_TIMES
while read -r line; do
    name=$(echo "$line" | cut -d ' ' -f 1)
    time=$(echo "$line" | cut -d ' ' -f 2)
    FILES_AND_TIMES["$name"]=$time
done < /home/magi/UAMdeconflictionMasterThesis/modelli/moreIterations.txt

# Trova i file .dat nella directory "data/" che corrispondono ai nomi in "moreIterations"
FILES=()
TIMES=()
for file in /home/magi/UAMdeconflictionMasterThesis/modelli/data/*.dat; do
    filename=$(basename "$file" .dat)
    if [[ -n "${FILES_AND_TIMES[$filename]}" ]]; then
        FILES+=("$file")
        TIMES+=("${FILES_AND_TIMES[$filename]}")
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

# Seleziona il file e il tempo corrispondente all'indice dell'array di Slurm
datFile=${FILES[$SLURM_ARRAY_TASK_ID]}
timeLimit=${TIMES[$SLURM_ARRAY_TASK_ID]}

# Ottieni il percorso assoluto del file corrente
datFileBase=$(basename "$datFile" .dat)

# Esegui AMPL con il file .dat corrente e il tempo massimo
timeLimit="$timeLimit" nNodes="10" nFlights="100" absPath=$PWD datFile=$datFileBase ampl /home/magi/UAMdeconflictionMasterThesis/modelli/fixedPath.run
