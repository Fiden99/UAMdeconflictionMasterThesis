for topology in {1..3}; do
    for seed in {0..5}; do
        #python3 /home/magi/UAMdeconflictionMasterThesis/pelegrin/Code\ Mercedes/parser_Magi.py $topology $seed 100
        nDrift=0
        for nDelay in {1..3}; do
            python3 /home/magi/UAMdeconflictionMasterThesis/pelegrin/Code\ Mercedes/scenario123-TDMagiFixed.py $topology $seed $nDrift $nDelay 0 0

        done
        nDrift=1
        for nDelay in {0..1}; do
            python3 /home/magi/UAMdeconflictionMasterThesis/pelegrin/Code\ Mercedes/scenario123-TDMagiFixed.py $topology $seed $nDrift $nDelay 0 0
        done
        python3 /home/magi/UAMdeconflictionMasterThesis/pelegrin/Code\ Mercedes/scenario123-TDMagiFixed.py $topology $seed 0 0 1 0
        python3 /home/magi/UAMdeconflictionMasterThesis/pelegrin/Code\ Mercedes/scenario123-TDMagiFixed.py $topology $seed 0 0 0 1
    done
done

#for topology in {1..3}; do
#    for seed in {6..19}; do
#        python3 /home/magi/UAMdeconflictionMasterThesis/pelegrin/Code\ Mercedes/scenario123-TDMagiFixed.py $topology $seed 0 0 1 0
#        python3 /home/magi/UAMdeconflictionMasterThesis/pelegrin/Code\ Mercedes/scenario123-TDMagiFixed.py $topology $seed 0 0 0 1
#    done
#done
#sbatch /home/magi/UAMdeconflictionMasterThesis/modelli/UAM_tactical_script.sh