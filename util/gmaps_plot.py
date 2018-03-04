import gmplot
from util.location import local_to_global


class GMapsPlot:
	def __init__(self, origin=(42.444376, -76.483504), scale=21):
		self.origin = origin
		self.map = gmplot.GoogleMapPlotter(origin[0], origin[1], scale)
		self.plots = []


	def add_plot(self, x_list, y_list, color='#222222'):
		lats = []
		lons = []

		for x, y in zip(x_list, y_list):
			lat, lon = local_to_global(x, y)
			lats.append(lat)
			lons.append(lon)

		self.plots.append({
			'lats':		lats,
			'lons': 	lons,
			'color':	color
		})


	def render(self, output_file='local/output/gmaps_plot.html'):
		for plot in self.plots:
			self.map.scatter(
				plot['lats'], 
				plot['lons'], 
				plot['color'],
				size=0.20,
				marker=False
			)

		self.map.draw(output_file)
