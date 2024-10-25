#!/bin/bash
#SBATCH --job-name=heuristicUAM_array_job       # Nome del job
#SBATCH --output=/home/magi/UAMdeconflictionMasterThesis/modelli/out/ampl_output_%A_%a.txt  # File di output per stdout (%A = job ID, %a = array task ID)
#SBATCH --error=/home/magi/UAMdeconflictionMasterThesis/modelli/out/ampl_error_%A_%a.txt    # File di output per stderr
#SBATCH --ntasks=1                      # Numero di task per job
#SBATCH --cpus-per-task=1               # Numero di core per job
#SBATCH --time=96:00:00                 # Tempo massimo per ogni job 
#SBATCH --array=0-54

# Crea una lista di tutti i file .dat nella directory "data/"
FILES=($(ls /home/magi/UAMdeconflictionMasterThesis/modelli/data/*.dat))
# FILES=(/home/magi/UAMdeconflictionMasterThesis/modelli/data/*.dat)

# Definisci il job array in base al numero di file presenti nella directory data/
NUM_FILES=${#FILES[@]}

# Carica il modulo AMPL (se necessario, se AMPL è gestito tramite moduli)
# module load ampl

# Seleziona il file corrispondente all'indice dell'array di Slurm
datFile=${FILES[$SLURM_ARRAY_TASK_ID]}

# Ottieni il percorso assoluto del file corrente
datFileBase=$(basename "$datFile" .dat)

# Esegui AMPL con il file .dat corrente
value="1" absPath=$PWD datFile=$datFileBase ampl /home/magi/UAMdeconflictionMasterThesis/modelli/heuristicAlgo.run
