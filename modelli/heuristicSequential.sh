#!/bin/bash
#SBATCH --job-name=heuristicUAM_seq_job       # Nome del job
#SBATCH --output=/home/magi/UAMdeconflictionMasterThesis/modelli/out/nn10nf100/ampl_output_%j.txt  # File di output per stdout (%j = job ID)
#SBATCH --error=/home/magi/UAMdeconflictionMasterThesis/modelli/out/nn10nf100/ampl_error_%j.txt    # File di output per stderr
#SBATCH --ntasks=1                      # Numero di task per job
#SBATCH --cpus-per-task=1               # Numero di core per job
#SBATCH --time=96:00:00                 # Tempo massimo per ogni job 

# Crea una lista di tutti i file .dat nella directory "data/"
FILES=($(ls /home/magi/UAMdeconflictionMasterThesis/modelli/data/*.dat))

# Itera sui file in modo sequenziale
for datFile in "${FILES[@]}"; do
    # Ottieni il nome del file senza l'estensione .dat
    datFileBase=$(basename "$datFile" .dat)
    
    # Esegui AMPL con il file .dat corrente
    value="7" nNodes="10" nFlights="100" absPath=$PWD datFile=$datFileBase ampl /home/magi/UAMdeconflictionMasterThesis/modelli/reducedHeuristicAlgo.run
done
