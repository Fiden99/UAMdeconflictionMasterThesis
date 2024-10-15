for i in ${PWD}/data/*.dat; do
        datFile=$(basename "$i" .dat)
        echo "$datFile"
        absPath=$PWD datFile="$datFile" ampl heuristicAlgo.run
done



