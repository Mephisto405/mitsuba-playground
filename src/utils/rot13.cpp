#include <mitsuba/render/util.h>
#include <mitsuba/core/sched.h>
MTS_NAMESPACE_BEGIN

class ROT13WorkUnit : public WorkUnit {
public:
	/// Copy the content of another work unit of the same type
	void set(const WorkUnit *workUnit) {
		const ROT13WorkUnit *wu =
			static_cast<const ROT13WorkUnit *>(workUnit);
		m_char = wu->m_char;
		m_pos = wu->m_pos;
	}
	/// Fill the work unit with content acquired from a binary data stream
	void load(Stream *stream) {
		m_char = stream->readChar();
		m_pos = stream->readInt();
	}
	/// Serialize a work unit to a binary data stream
	void save(Stream *stream) const {
		stream->writeChar(m_char);
		stream->writeInt(m_pos);
	}
	/// Return a string representation
	std::string toString() const {
		std::ostringstream oss;
		oss << "ROT13WorkUnit[" << endl
			<< " char = '" << m_char << "'," << endl
			<< " pos = " << m_pos << endl
			<< "]";
		return oss.str();
	}

	inline char getChar() const { return m_char; }
	inline void setChar(char value) { m_char = value; }
	inline int getPos() const { return m_pos; }
	inline void setPos(int value) { m_pos = value; }

	MTS_DECLARE_CLASS()
private:
	char m_char;
	int m_pos;
};

class ROT13WorkResult : public WorkResult {
public:
	/// Fill the work result with content acquired from a binary data stream
	void load(Stream *stream) {
		m_char = stream->readChar();
		m_pos = stream->readInt();
	}
	/// Serialize a work result to a binary data stream
	void save(Stream *stream) const {
		stream->writeChar(m_char);
		stream->writeInt(m_pos);
	}
	/// Return a string representation
	std::string toString() const {
		std::ostringstream oss;
		oss << "ROT13WorkResult[" << endl
			<< " char = '" << m_char << "'," << endl
			<< " pos = " << m_pos << endl
			<< "]";
		return oss.str();
	}

	inline char getChar() const { return m_char; }
	inline void setChar(char value) { m_char = value; }
	inline int getPos() const { return m_pos; }
	inline void setPos(int value) { m_pos = value; }

	MTS_DECLARE_CLASS()
private:
	char m_char;
	int m_pos;
};

/// we need a class, which does the actual work of turning 
/// a work unit into a work result
class ROT13WorkProcessor : public WorkProcessor {
public:
	/// Construct a new work processor
	ROT13WorkProcessor() : WorkProcessor() { }
	/// Unserialize from a binary data stream (nothing to do in our case)
	ROT13WorkProcessor(Stream *stream, InstanceManager *manager)
		: WorkProcessor(stream, manager) { }
	/// Serialize to a binary data stream (nothing to do in our case)
	void serialize(Stream *stream, InstanceManager *manager) const {
	}


	ref<WorkUnit> createWorkUnit() const {
		return new ROT13WorkUnit();
	}
	ref<WorkResult> createWorkResult() const {
		return new ROT13WorkResult();
	}
	/// Work processor instances will be replicated amongst(among) local threads
	ref<WorkProcessor> clone() const {
		/**
		* \brief Create a copy of this work processor instance.
		*
		* \remark In practice, before the cloned work processor
		* is actually used, its \ref prepare() method will be called.
		* Therefore, any state that is initialized in \ref prepeare()
		* does not have to be copied.
		*/
		return new ROT13WorkProcessor(); // No state to clone in our case
	}
	/// No internal state, thus no preparation is necessary
	void prepare() { }

	/// Do the actual computation
	void process(const WorkUnit *workUnit, WorkResult *workResult,
		const bool &stop) {
		/// WorkUnit 에서 input들을 불러오고
		const ROT13WorkUnit *wu
			= static_cast<const ROT13WorkUnit *>(workUnit);
		/// WorkResult 에 계산 결과를 집어넣는다.
		ROT13WorkResult *wr = static_cast<ROT13WorkResult *>(workResult);
		wr->setPos(wu->getPos());
		wr->setChar((std::toupper(wu->getChar()) - 'A' + 13) % 26 + 'A');
	}
	MTS_DECLARE_CLASS()
};

/// Parallel Process instance is responsible for creating work units 
/// and stitching work results back into a solution of the whole problem
class ROT13Process : public ParallelProcess {
public:
	// Initialize
	ROT13Process(const std::string &input) : m_input(input), m_pos(0) {
		m_output.resize(m_input.length());
	}

	// Takes a pre-allocated \ref WorkUnit instance of
	// the appropriate sub - type and size and
	// fills it with the appropriate content.
	EStatus generateWork(WorkUnit *unit, int worker) {
		if (m_pos >= (int) m_input.length())
			return EFailure;
		ROT13WorkUnit *wu = static_cast<ROT13WorkUnit *>(unit);

		wu->setPos(m_pos);
		/// pre-allocated 된 WorkUnit을 하나 하나 채울 때 마다 pos를 늘려준다.
		/// ESuccess는 다음 work를 만들어서 실행한다 
		/// (Scheduler::acquireWork(...) 와 LocalWorker::run(...) 참고)
		/// EFailure가 뜰 때까지 반복 수행한다.
		wu->setChar(m_input[m_pos++]);
		
		return ESuccess;
	}

	ref<WorkProcessor> createWorkProcessor() const {
		return new ROT13WorkProcessor();
	}

	// 각 process의 결과가 어떻게 전체 결과물에 공헌을 할지
	void processResult(const WorkResult *result, bool cancelled) {
		if (cancelled)
			return;
		const ROT13WorkResult *wr = static_cast<const ROT13WorkResult *>(result);

		m_output[wr->getPos()] = wr->getChar();
	}

	std::vector<std::string> getRequiredPlugins() {
		std::vector<std::string> result;
		result.push_back("rot13");
		return result;
	}

	inline std::string getOutput() { return m_output; }

	MTS_DECLARE_CLASS()
public:
	std::string m_input;
	std::string m_output;
	int m_pos;
};


class ROT13Encoder : public Utility {
public:
	int run(int argc, char **argv) {

		if (argc < 2) {
			cout << "[Error!] Syntax : mtstutil rot13 <text>" << endl;
			return -1;
		}

		/// String 하나를 인풋으로 건네주고 
		/// output으로는 변환된 string을 받아야 한다
		ref<ROT13Process> proc = new ROT13Process(argv[1]);
		ref<Scheduler> sched = Scheduler::getInstance();

		sched->schedule(proc);
		sched->wait(proc);

		cout << "Result: " << proc->getOutput() << endl;
		
		float *m_pointer;
		cout << "Size of float pointer: " << sizeof(m_pointer) << endl; // 8

		return 0;

	}
	MTS_DECLARE_UTILITY()
};

MTS_IMPLEMENT_CLASS(ROT13Process, false, ParallelProcess)
MTS_IMPLEMENT_CLASS_S(ROT13WorkProcessor, false, WorkProcessor)
MTS_IMPLEMENT_CLASS(ROT13WorkResult, false, WorkResult)
MTS_IMPLEMENT_CLASS(ROT13WorkUnit, false, WorkUnit)
MTS_EXPORT_UTILITY(ROT13Encoder, "Perform a ROT13 encryption of a string")
MTS_NAMESPACE_END