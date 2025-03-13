#!/bin/bash
#SBATCH --job-name=heuristicUAM_array_job       # Nome del job
#SBATCH --output=/home/magi/UAMdeconflictionMasterThesis/modelli/out/ampl_output_%A_%a.txt  # File di output per stdout (%A = job ID, %a = array task ID)
#SBATCH --error=/home/magi/UAMdeconflictionMasterThesis/modelli/out/ampl_error_%A_%a.txt    # File di output per stderr
#SBATCH --ntasks=1                      # Numero di task per job
#SBATCH --cpus-per-task=1               # Numero di core per job
#SBATCH --time=96:00:00                 # Tempo massimo per ogni job 

# Crea una lista di tutti i file .dat nella directory "data/"

# Esegui AMPL con il file .dat corrente
value="6" absPath=$PWD datFile="airport0" ampl /home/magi/UAMdeconflictionMasterThesis/modelli/reducedHeuristicAlgo.run
