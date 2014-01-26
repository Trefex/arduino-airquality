setwd('sketchbook/arduino-airquality/Module_COSensor/outputs/')
plot_co_data('output.txt')
plot_co_data('output2.txt')
plot_co_data('output3.txt')
plot_co_data('output4.txt')
plot_co_data('output5.txt')
data <- plot_co_data('output6.txt')

plot_co_data('li-ion_output.txt')
plot_co_data('butane_exposure.txt')

burn <- plot_co_data('burning.txt')
plot(burn$value, type='l')

plot_co_data <- function(fn) {
  dict <- list(C = 'Cooling', H = 'Heating', I='Initialization', R='Reading')
  
  fd <- read.csv(fn, header=FALSE)
  names(fd) <- c('type', 'value')
  plot(fd$value, col=fd$type, cex=0.7)
  legend('topleft', legend=dict[names(table(fd$type))], col=c(1:4), pch=1, cex=0.7)
  return(fd)
}

libdata <- read.table('output7.txt',sep=',')
names(libdata) <- c('mode', 'voltage','CO')
plot(libdata[libdata$mode == 4,]$CO)
plot(libdata$CO, cex=0.5)

burning <- read.table('burning.txt', sep=',')

plot(burning$V2, type='l', xlab='Time (seconds)', ylab='Reading (resistance Ω)', main='Readings from first 70 minutes of cooking sensor')

post_burning <- read.table('post_burn_output.txt', sep=',')
plot(post_burning$V3, col=post_burning$V1, cex=0.3)
plot(post_burning[post_burning$V1 == 4,3], xlab='Time (interval of 150 seconds)', ylab='CO resistance reading')
