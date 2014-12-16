# Eric Schneider

import json

if __name__ == '__main__':
    data = []
    data.append((99.7, 97.92, 'up'))
    data.append((100.3, 99.52, 'up'))

    with open('data_12_15_14.txt', 'w') as outfile:
        json.dump(data, outfile)
