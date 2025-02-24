#!/bin/bash
#SBATCH --job-name=createInstance
#SBATCH --output=out/output_%A_%a.log   # Log separato per ogni job
#SBATCH --error=out/error_%A_%a.log     # Log errori per ogni job
#SBATCH --array=0-299%50                   # 300 job paralleli (3 topologie × 100 seed)
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1               # 1 CPU per job

set -eu  # Evita errori nascosti e variabili non definite
#da controllare eventualemnte qui
topology=$((SLURM_ARRAY_TASK_ID / 100 + 1))
seed=$((SLURM_ARRAY_TASK_ID % 100))


#python3 /home/magi/UAMdeconflictionMasterThesis/pelegrin/Code\ Mercedes/parser_Magi.py $topology $seed 100
for nDelay in {1..3}; do
    python3 /home/magi/UAMdeconflictionMasterThesis/pelegrin/Code\ Mercedes/scenario123Heur.py $topology $seed 0 $nDelay 0 0
done
python3 /home/magi/UAMdeconflictionMasterThesis/pelegrin/Code\ Mercedes/scenario123Heur.py $topology $seed 0 0 1 0
python3 /home/magi/UAMdeconflictionMasterThesis/pelegrin/Code\ Mercedes/scenario123Heur.py $topology $seed 0 0 0 1

