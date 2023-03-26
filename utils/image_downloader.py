from simple_image_download import simple_image_download as sim

response = sim.simple_image_download

keywords = ["calculator", "mobile", "bottle", "bolt"]

for keyword in keywords:
    response().download(keyword, 5)
