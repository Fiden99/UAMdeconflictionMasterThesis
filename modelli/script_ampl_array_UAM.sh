#!/bin/bash
#SBATCH --job-name=ampl_array_job       # Nome del job
#SBATCH --output=/home/magi/models/out/ampl_output_%A_%a.txt  # File di output per stdout (%A = job ID, %a = array task ID)
#SBATCH --error=/home/magi/models/out/ampl_error_%A_%a.txt    # File di output per stderr
#SBATCH --ntasks=1                      # Numero di task per job
#SBATCH --time=48:00:00                 # Tempo massimo per ogni job (30 minuti, adattalo alle tue necessità)
#SBATCH --mem=8GB                       # Memoria per ogni job (adattala alle tue necessità)

# Definisci il job array in base al numero di file presenti nella directory data/
NUM_FILES=$(ls /home/magi/models/data/*.dat |wc -l)
#SBATCH --array=0-$(($NUM_FILES -1))
#SBATCH --array=0-0   # Lancia solo il primo job nell'array
# Carica il modulo AMPL (se necessario, se AMPL è gestito tramite moduli)
module load ampl

# Crea una lista di tutti i file .dat nella directory "data/"
FILES=(/home/magi/models/data/*.dat)

# Seleziona il file corrispondente all'indice dell'array di Slurm
datFile=${FILES[$SLURM_ARRAY_TASK_ID]}

# Estrai il nome base del file senza l'estensione .dat
datFileBase=$(basename "$datFile" .dat)

# Stampa il file che viene processato (utile per il debugging)
echo "Processing file: $datFile"

# Esegui AMPL con il file .dat corrente
datFile="$datFileBase" ampl /home/magi/models/UAMserver.run

