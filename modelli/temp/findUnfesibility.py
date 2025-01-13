import pandas as pd

df = pd.read_csv('t_ear.csv')

table = df[df.columns[1:-1]]
for column in table.columns:
    for row1 in range(0, len(df)):
        for row2 in range(0, len(df)):
            if abs(table[column].iloc[row1] - table[column].iloc[row2]) < 2 and table[column].iloc[row1] != 0 and table[column].iloc[row2] != 0 and row1 != row2:
                print(column, row1, row2)