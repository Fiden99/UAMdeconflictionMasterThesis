for topology in {1..3}; do
    for seed in {0..5}; do
        #python3 /home/magi/UAMdeconflictionMasterThesis/pelegrin/Code\ Mercedes/parser_Magi.py $topology $seed
        nDrift=0
        for nDelay in {1..3}; do
            python3 /home/magi/UAMdeconflictionMasterThesis/pelegrin/Code\ Mercedes/scenario123TD_Magi.py $topology $seed $nDrift $nDelay
        done
        nDrift=1
        for nDelay in {0..1}; do
            python3 /home/magi/UAMdeconflictionMasterThesis/pelegrin/Code\ Mercedes/scenario123TD_Magi.py $topology $seed $nDrift $nDelay
        done
    done
done
#sbatch /home/magi/UAMdeconflictionMasterThesis/modelli/UAM_tactical_script.sh