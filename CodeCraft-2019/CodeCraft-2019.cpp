#include "common.h"
#include "graph.h"
#include "scheduler.h"

int main(int argc, char *argv[]) {
    cout << "Begin" << std::endl;

	if(argc < 5){
		std::cout << "please input args: carPath, roadPath, crossPath, answerPath" << std::endl;
		exit(1);
	}

	string carPath(argv[1]);
	string roadPath(argv[2]);
	string crossPath(argv[3]);
	string presetAnswerPath(argv[4]);
	string answerPath(argv[5]);

	cout << "carPath is " << carPath << std::endl;
	cout << "roadPath is " << roadPath << std::endl;
	cout << "crossPath is " << crossPath << std::endl;
	cout << "presetAnswerPath is " << presetAnswerPath << endl;
	cout << "answerPath is " << answerPath << std::endl;

    // TODO:read input filebuf
	ifstream carStream, roadStream, crossStream, presetAnswerStream;
    ofstream answerStream;
	carStream.open(carPath, ios::in);
	if (carStream.fail()) {
		cout << "fail to read carPath" << endl;
		assert(false);
	}

	roadStream.open(roadPath, ios::in);
	if (roadStream.fail()) {
		cout << "fail to read roadPath" << endl;
		assert(false);
	}

	crossStream.open(crossPath, ios::in);
	if (crossStream.fail()) {
		cout << "fail to read crossPath" << endl;
		assert(false);
	}

	presetAnswerStream.open(presetAnswerPath, ios::in);
	if (presetAnswerStream.fail()) {
		cout << "fail to read presetAnswerPath" << endl;
		assert(false);
	}

    answerStream.open(answerPath, ios::out);
	if (answerStream.fail()) {
		cout << "fail to read answerPath" << endl;
		assert(false);
	}

	auto scheduler = new Scheduler(carStream, roadStream, crossStream, presetAnswerStream);

	cout << "Begin simulating" << endl;
	scheduler->changeTenPercent();
    scheduler->simulate();
	scheduler->outputAnswer(answerStream);
	return 0;
}
