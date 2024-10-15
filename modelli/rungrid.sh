
for i in $PWD/data/grid*.dat; do
        datFile=$(basename "$i" .dat)
        echo "$datFile"
        absPath=$PWD datFile="$datFile" ampl heuristicAlgo.run
done




