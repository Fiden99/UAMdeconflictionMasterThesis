for i in data/grid*.dat; do
        datFile=$(basename "$i" .dat)
        echo "$datFile"
        datFile="$datFile" ampl heuristicAlgo.run
done




