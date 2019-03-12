///m_film_depth

#include <mitsuba/core/statistics.h>
#include <mitsuba/core/sfcurve.h>
#include "wlrproc.h"
#include <mitsuba/render/rectwu.h>

MTS_NAMESPACE_BEGIN
/// WorkUnit = RectangularWorkUnit

/// WorkResult = ImageBlock

/// WorkProcessor
/// we need a class, which does the actual work of turning 
/// a work unit into a work result
class BlockWLRRenderer : public WorkProcessor {
public:
	BlockWLRRenderer(Bitmap::EPixelFormat pixelFormat, int channelCount, int blockSize,
		int borderSize, bool warnInvalid) : m_pixelFormat(pixelFormat),
		m_channelCount(channelCount), m_blockSize(blockSize),
		m_borderSize(borderSize), m_warnInvalid(warnInvalid) { }

	void process(/*....*/) {
		/// 각 프로세스는 renderBlock을 수행한다.
	}

private:
	ref<Scene> m_scene;
	ref<Sensor> m_sensor;
	ref<Sampler> m_sampler;
	ref<SamplingIntegrator> m_integrator;
	Bitmap::EPixelFormat m_pixelFormat;
	int m_channelCount;
	int m_featureChannelCount; // Edited
	int m_blockSize;
	int m_borderSize;
	bool m_warnInvalid;
	HilbertCurve2D<uint8_t> m_hilbertCurve;
};







/// ParallelProcess = BlockedWLRProcess wlrproc.h
BlockedWLRProcess::BlockedWLRProcess(const RenderJob *parent, RenderQueue *queue,
	int blockSize) : m_queue(queue), m_parent(parent), m_resultCount(0), m_progress(NULL) {
	m_blockSize = blockSize;
	m_resultMutex = new Mutex();
	m_pixelFormat = Bitmap::ESpectrumAlphaWeightDepth;
	m_channelCount = -1;
	m_warnInvalid = true;
}

BlockedWLRProcess::~BlockedWLRProcess() {
	if (m_progress)
		delete m_progress;
}

void BlockedWLRProcess::setPixelFormat(Bitmap::EPixelFormat pixelFormat, int channelCount, bool warnInvalid) {
	m_pixelFormat = pixelFormat;
	m_channelCount = channelCount;
	m_warnInvalid = warnInvalid;
}

ref<WorkProcessor> BlockedWLRProcess::createWorkProcessor() const {
	return new BlockWLRRenderer(m_pixelFormat, m_channelCount,
		m_blockSize, m_borderSize, m_warnInvalid);
}

// 각 process의 결과가 어떻게 전체 결과물에 공헌을 할지
void BlockedWLRProcess::processResult(const WorkResult *result, bool cancelled) {
	const ImageBlock *block = static_cast<const ImageBlock *>(result);
	UniqueLock lock(m_resultMutex);
	/// 전체 film에 block의 결과물을 주입한다 (block의 index는 block안에 저장되어있는 정보다.)
	
	m_film->put(block);

	m_progress->update(++m_resultCount);
	lock.unlock();
	m_queue->signalWorkEnd(m_parent, block, cancelled);
}


//MTS_IMPLEMENT_CLASS(BlockedWLRProcess, false, BlockedImageProcess)
//MTS_IMPLEMENT_CLASS_S(BlockWLRRenderer, false, WorkProcessor)
MTS_NAMESPACE_END