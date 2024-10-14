for i in data/*.dat; do
        datFile=$(basename "$i" .dat)
        echo "$datFile"
        datFile="$datFile" ampl heuristicAlgo.run
done



