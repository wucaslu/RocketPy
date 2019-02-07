import cdsapi

c = cdsapi.Client()

years = ['2015', '2016', '2017', '2018']

for year in years:
    print()
    print(year)
    print()
    c.retrieve(
        'reanalysis-era5-pressure-levels',
        {
            'product_type':'reanalysis',
            'format':'netcdf',
            'variable':[
                'geopotential','temperature','u_component_of_wind',
                'v_component_of_wind'
            ],
            'pressure_level':[
                '600',
                '650','700','750',
                '775','800','825',
                '850','875','900',
                '925','950','975',
                '1000'
            ],
            'year':[
                year
            ],
            'month':[
                '01','02','03',
                '04','05','06',
                '07','08','09',
                '10','11','12'
            ],
            'day':[
                '01','02','03',
                '04','05','06',
                '07','08','09',
                '10','11','12',
                '13','14','15',
                '16','17','18',
                '19','20','21',
                '22','23','24',
                '25','26','27',
                '28','29','30',
                '31'
            ],
            'time':[
                '00:00','06:00','12:00',
                '18:00'
            ],
            'area': '-4.5/324/-7.5/326.25'
        },
        'CLBI_' + year + '_ERA-5.nc')