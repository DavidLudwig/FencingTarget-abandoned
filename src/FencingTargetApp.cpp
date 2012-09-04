#include <algorithm>
#include <fstream>
#include <iterator>
using namespace std;

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
//using namespace boost;
using namespace boost::asio;

#include "cinder/app/AppBasic.h"
#include "cinder/gl/gl.h"
#include "cinder/params/Params.h"
using namespace ci;
using namespace ci::app;

//static const string FORMAT_OF_ACCELEROMETER = "(\\d+),(\\d+),(\\d+)";
//static const string INPUT_FILE = "/Users/davidl/Documents/Code/cinder/FencingTarget/FencingTarget/assets/shortfakedata.dat";

static const string FORMAT_OF_ACCELEROMETER = "acc:(\\d+),(\\d+),(\\d+)";
//static const string INPUT_FILE = "/dev/tty.usbmodem1411";	// left USB port
static const string INPUT_FILE = "/dev/tty.usbmodem1421";	// right USB port

static const size_t NUM_ACCELEROMETER_READINGS = 600;


class FencingTargetApp : public AppBasic {
public:
	FencingTargetApp();
	void prepareSettings(Settings * settings);
	void setup();
	void update();
	void drawAccelerometerReadings(boost::function<float (Vec3f)> getValueGivenAccelReading);
	void draw();
	virtual void keyDown(KeyEvent event);
	
private:
	// Accelerometer-reading stuff:
	void requestAsyncReadFromArduino();
	void handleInputFromArduino(const boost::system::error_code & errorCode);
	static bool parseLineOfRawAccelerometerData(const string & s, cinder::Vec3f & out);
	boost::asio::io_service io_;
	boost::asio::serial_port port_;
	boost::asio::streambuf buffer_;
	
	// Parsed data from the accelerometer:
	boost::circular_buffer<Vec3f> rawAccelerometerReadings_;
	Vec3f transformReading(Vec3f rawReading) const;

	// Adjustable parameters, Data:
	float pre_offset_;
	float post_scale_;
	
	// Adjustable parameters, GUI:
	cinder::params::InterfaceGl params_gui_;
	float draw_scale_;
};

FencingTargetApp::FencingTargetApp() : port_(io_) {
}

void FencingTargetApp::prepareSettings(Settings * settings) {
	settings->setWindowSize(1024, 768);
	settings->setFrameRate(60.0f);
}

void FencingTargetApp::setup() {
	// Set aside enough data for accelerometer readings:
	rawAccelerometerReadings_.set_capacity(NUM_ACCELEROMETER_READINGS);
	
	// Initialize the serial port:
	try {
		port_.open(INPUT_FILE);
	} catch (std::exception & e) {
		cerr << "ERROR: Unable to open device at '" << INPUT_FILE << "', error=\"" << e.what() << "\"\n";
	}
	// TODO: make sure INPUT_FILE was opened correctly.  If not, fail gracefully.
	if (port_.is_open()) {
		port_.set_option(serial_port::baud_rate(115200));
		port_.set_option(serial_port::character_size(8));
		port_.set_option(serial_port::flow_control(serial_port::flow_control::none));
		port_.set_option(serial_port::parity(serial_port::parity::none));
		port_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
		
		// Request data from the Arduino:
		requestAsyncReadFromArduino();
	}
	
	// Init parameters, data:
	pre_offset_ = -128.0f;
	post_scale_ = 1.0f;
	
	// Init parameters, GUI:
	draw_scale_ = -1.0;
	
	// Init the parameter-changing GUI:
	params_gui_ = cinder::params::InterfaceGl("Parameters", cinder::Vec2i(250, 500));

	params_gui_.addText("DATA, PRE:");
	params_gui_.addParam("Offset", &pre_offset_, "min=-300 max=300 step=1");

	params_gui_.addSeparator();
	params_gui_.addText("DATA, POST:");
	params_gui_.addParam("Scale", &post_scale_, "min=0.01 max=2.0 step=0.01");
	
	params_gui_.addSeparator();
	params_gui_.addText("DRAWING:");
	params_gui_.addParam("Draw Scale", &draw_scale_, "min=-40.0 max=-0.01 step=0.01");
	//params_gui_.hide();
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
		rawAccelerometerReadings_.push_back(accelerometerReading);
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
	if (port_.is_open()) {
		io_.poll();
	}
}

Vec3f FencingTargetApp::transformReading(Vec3f rawReading) const {
	Vec3f pre_offset_vector(pre_offset_, pre_offset_, pre_offset_);
	return (rawReading + pre_offset_vector) * post_scale_;
}

void FencingTargetApp::drawAccelerometerReadings(boost::function<float (Vec3f)> getValueGivenAccelReading) {
	gl::begin(GL_TRIANGLE_STRIP);
	int i = 0;
	BOOST_FOREACH(Vec3f fullReading, rawAccelerometerReadings_) {
		gl::vertex(i, 0);
		const Vec3f transformedReading = this->transformReading(fullReading);
		const float interpretedReading = getValueGivenAccelReading(transformedReading);
		gl::vertex(i, draw_scale_ * interpretedReading);
		i++;
	}
	gl::end();
}

void FencingTargetApp::draw() {
	// Clear the window:
	gl::clear(Color::white());
	
	// Draw graphs:
	gl::pushMatrices();
	gl::translate(350, 0);

	// Draw X:
	gl::color(Color8u::hex(0xFF0000));
	gl::pushMatrices();
	gl::translate(0, 150);
	drawAccelerometerReadings(boost::bind(&Vec3f::x, boost::lambda::_1));
	gl::popMatrices();

	// Draw Y:
	gl::color(Color8u::hex(0x00FF00));
	gl::pushMatrices();
	gl::translate(0, 300);
	drawAccelerometerReadings(boost::bind(&Vec3f::y, boost::lambda::_1));
	gl::popMatrices();

	// Draw Z:
	gl::color(Color8u::hex(0x0000FF));
	gl::pushMatrices();
	gl::translate(0, 450);
	drawAccelerometerReadings(boost::bind(&Vec3f::z, boost::lambda::_1));
	gl::popMatrices();
	
	// Draw length:
	gl::color(Color8u::hex(0x000000));
	gl::pushMatrices();
	gl::translate(0, 700);
	drawAccelerometerReadings(boost::bind(&Vec3f::length, boost::lambda::_1));
	gl::popMatrices();
	
	// Finish drawing graphs:
	gl::popMatrices();

	// Draw the gui of parameters:
	params::InterfaceGl::draw();
}

void FencingTargetApp::keyDown(KeyEvent event) {
	switch (event.getCode()) {
		case KeyEvent::KEY_SPACE:
			params_gui_.show( ! params_gui_.isVisible() );
			break;
		
		default:
			break;
	}
}


CINDER_APP_BASIC( FencingTargetApp, RendererGl )
