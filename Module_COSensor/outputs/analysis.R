setwd('sketchbook/arduino-airquality/Module_COSensor/outputs/')
plot_co_data('output.txt')
plot_co_data('output2.txt')
plot_co_data('output3.txt')
plot_co_data('output4.txt')
plot_co_data('output5.txt')
plot_co_data('li-ion_output.txt')


plot_co_data('butane_exposure.txt')
burn <- plot_co_data('burning.txt')
plot(burn$value, type='l')

plot_co_data <- function(fn) {
  fd <- read.csv(fn, header=FALSE)
  names(fd) <- c('type', 'value')
  plot(fd$value, col=fd$type)
  legend('topleft', legend=names(table(fd$type)), col=c(1:3), pch=1)
  return(fd)
}