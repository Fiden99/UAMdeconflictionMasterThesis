#!/bin/bash
#SBATCH --job-name=testFixOrder      # Nome del job
#SBATCH --ntasks=1                      # Numero di task per job
#SBATCH --cpus-per-task=1               # Numero di core per job
#SBATCH --time=96:00:00                 # Tempo massimo per ogni job 
#SBATCH --array=0-500%50                # Numero di job da eseguire
#SBATCH --output=/dev/null
#SBATCH --error=/dev/null


# Crea una lista di tutti i file .dat nella directory "data/"
FILES=($(ls /home/magi/UAMdeconflictionMasterThesis/modelli/data/mercedesTD/AP/*.dat))
# FILES=(/home/magi/UAMdeconflictionMasterThesis/modelli/data/*.dat)

# Seleziona il file corrispondente all'indice dell'array di Slurm
datFile=${FILES[$SLURM_ARRAY_TASK_ID]}

# Ottieni il percorso assoluto del file corrente
datFileBase=$(basename "$datFile" .dat)

# Esegui AMPL con il file .dat corrente
absPath=$PWD datFile=$datFileBase ampl /home/magi/UAMdeconflictionMasterThesis/modelli/testFixOrder.run

