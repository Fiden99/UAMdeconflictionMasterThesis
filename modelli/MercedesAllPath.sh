#!/bin/bash
#SBATCH --job-name=MercedesInstances     # Nome del job
#SBATCH --output=/home/magi/UAMdeconflictionMasterThesis/modelli/out/mercedesAllPath/ampl_output_%A_%a.txt  # File di output per stdout (%A = job ID, %a = array task ID)
#SBATCH --error=/home/magi/UAMdeconflictionMasterThesis/modelli/out/mercedesAllPath/ampl_error_%A_%a.txt    # File di output per stderr
#SBATCH --ntasks=1                      # Numero di task per job
#SBATCH --cpus-per-task=1               # Numero di core per job
#SBATCH --time=96:00:00                 # Tempo massimo per ogni job 
#SBATCH --array=0-50%10                    # Numero di job da eseguire

# Leggi la lista dei file da un file di testo
mapfile -t FILES < /home/magi/UAMdeconflictionMasterThesis/modelli/temp/file_list.txt
# FILES=(/home/magi/UAMdeconflictionMasterThesis/modelli/data/*.dat)

# Seleziona il file corrispondente all'indice dell'array di Slurm
datFile=${FILES[$SLURM_ARRAY_TASK_ID]}

# Ottieni il percorso assoluto del file corrente
#datFileBase=$(basename "$datFile" .dat)

echo "Running AMPL for $datFileBase"

# Esegui AMPL con il file .dat corrente
absPath=$PWD datFile=$datFile ampl /home/magi/UAMdeconflictionMasterThesis/modelli/MercedesAllPath.run
