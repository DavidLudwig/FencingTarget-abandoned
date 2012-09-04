#include <algorithm>
#include <fstream>
#include <iterator>
using namespace std;

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
//using namespace boost;
using namespace boost::asio;

#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
using namespace ci;
using namespace ci::app;

//static const string FORMAT_OF_ACCELEROMETER = "(\\d+),(\\d+),(\\d+)";
//static const string INPUT_FILE = "/Users/davidl/Documents/Code/cinder/FencingTarget/FencingTarget/assets/shortfakedata.dat";

static const string FORMAT_OF_ACCELEROMETER = "acc:(\\d+),(\\d+),(\\d+)";
//static const string INPUT_FILE = "/dev/tty.usbmodem1411";	// left USB port
static const string INPUT_FILE = "/dev/tty.usbmodem1421";	// right USB port

static const size_t NUM_ACCELEROMETER_READINGS = 800;


class FencingTargetApp : public AppBasic {
public:
	FencingTargetApp();
	void prepareSettings(Settings * settings);
	void setup();
	void update();
	void drawAccelerometerReadings(boost::function<float (Vec3f)> getValueGivenAccelReading);
	void draw();
	
private:
	// Accelerometer-reading stuff:
	void requestAsyncReadFromArduino();
	void handleInputFromArduino(const boost::system::error_code & errorCode);
	static bool parseLineOfRawAccelerometerData(const string & s, cinder::Vec3f & out);
	boost::asio::io_service io_;
	boost::asio::serial_port port_;
	boost::asio::streambuf buffer_;
	
	// Parsed data from the accelerometer:
	boost::circular_buffer<Vec3f> accelerometerReadings_;
};

FencingTargetApp::FencingTargetApp() : port_(io_) {
}

void FencingTargetApp::prepareSettings(Settings * settings) {
	settings->setWindowSize(1024, 768);
	settings->setFrameRate(60.0f);
}

void FencingTargetApp::setup() {
	// Set aside enough data for accelerometer readings:
	accelerometerReadings_.set_capacity(NUM_ACCELEROMETER_READINGS);
	
	// Initialize the serial port:
	port_.open(INPUT_FILE);
	// TODO: make sure INPUT_FILE was opened correctly.  If not, fail gracefully.
	port_.set_option(serial_port::baud_rate(115200));
	port_.set_option(serial_port::character_size(8));
	port_.set_option(serial_port::flow_control(serial_port::flow_control::none));
	port_.set_option(serial_port::parity(serial_port::parity::none));
	port_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
	
	// Request data from the Arduino:
	requestAsyncReadFromArduino();
}

void FencingTargetApp::requestAsyncReadFromArduino() {
	boost::asio::async_read_until(port_,
								  buffer_,
								  boost::regex("\r\n"),
								  boost::bind(&FencingTargetApp::handleInputFromArduino,
											  boost::ref(*this),
											  boost::lambda::_1));
}

void FencingTargetApp::handleInputFromArduino(const boost::system::error_code & errorCode)
{
	// Make the buffer's data accessible as a std::istream:
	std::istream is(&buffer_);
	
	// Set up a string to contain a line's worth of data:
	std::string line;
	
	// Attempt to read in one line's worth of data, pruning it from the
	// buffer in the process:
	std::getline(is, line);
	//	cout << "LINE: " << line;
	
	// Try parsing some accelerometer data:
	Vec3f accelerometerReading;
	if (parseLineOfRawAccelerometerData(line, accelerometerReading)) {
		cout << accelerometerReading << endl;
		accelerometerReadings_.push_back(accelerometerReading);
	}
	
	// Request more data from the Arduino:
	requestAsyncReadFromArduino();
}

bool FencingTargetApp::parseLineOfRawAccelerometerData(const string & s, cinder::Vec3f & out) {
	static boost::regex line_matcher(FORMAT_OF_ACCELEROMETER);
	boost::smatch match;
	if ( ! boost::regex_search(s.begin(), s.end(), match, line_matcher) ) {
		return false;
	}
	
	try {
		double x = boost::lexical_cast<double>(match[1].str());
		double y = boost::lexical_cast<double>(match[2].str());
		double z = boost::lexical_cast<double>(match[3].str());
		out = cinder::Vec3f(x, y, z);
		return true;
	} catch (boost::bad_lexical_cast &) {
		return false;
	}
}

void FencingTargetApp::update() {
	// Process incoming data from the Arduino, if any is available:
	io_.poll();
}

void FencingTargetApp::drawAccelerometerReadings(boost::function<float (Vec3f)> getValueGivenAccelReading) {
	gl::begin(GL_TRIANGLE_STRIP);
	int i = 0;
	BOOST_FOREACH(Vec3f fullReading, accelerometerReadings_) {
		gl::vertex(i, 0);
		const float interpretedReading = getValueGivenAccelReading(fullReading);
		gl::vertex(i, -0.5f * interpretedReading);
		i++;
	}
	gl::end();
}

void FencingTargetApp::draw() {
	// Clear the window:
	gl::clear(Color::white());

	// Draw X:
	gl::color(Color8u::hex(0xFF0000));
	gl::pushMatrices();
	gl::translate(100, 150);
	drawAccelerometerReadings(boost::bind(&Vec3f::x, boost::lambda::_1));
	gl::popMatrices();

	// Draw Y:
	gl::color(Color8u::hex(0x00FF00));
	gl::pushMatrices();
	gl::translate(100, 300);
	drawAccelerometerReadings(boost::bind(&Vec3f::y, boost::lambda::_1));
	gl::popMatrices();

	// Draw Z:
	gl::color(Color8u::hex(0x0000FF));
	gl::pushMatrices();
	gl::translate(100, 450);
	drawAccelerometerReadings(boost::bind(&Vec3f::z, boost::lambda::_1));
	gl::popMatrices();
	
	// Draw length:
	gl::color(Color8u::hex(0x000000));
	gl::pushMatrices();
	gl::translate(100, 700);
	drawAccelerometerReadings(boost::bind(&Vec3f::length, boost::lambda::_1));
	gl::popMatrices();

}


CINDER_APP_BASIC( FencingTargetApp, RendererGl )
