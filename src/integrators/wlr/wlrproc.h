#pragma once
#if !defined(__MITSUBA_RENDER_WLRPROC_H_)
#define __MITSUBA_RENDER_WLRPROC_H_

#include <mitsuba/render/scene.h>
#include <mitsuba/render/imageproc.h>
#include <mitsuba/render/renderqueue.h>

MTS_NAMESPACE_BEGIN

class MTS_EXPORT_RENDER BlockedWLRProcess : public BlockedImageProcess {
public:
	BlockedWLRProcess(const RenderJob *parent, RenderQueue *queue,
		int blockSize);

	void setPixelFormat(Bitmap::EPixelFormat pixelFormat,
		int channelCount = -1, bool warnInvalid = false);

	// ======================================================================
	//! @{ \name Implementation of the ParallelProcess interface
	// ======================================================================

	ref<WorkProcessor> createWorkProcessor() const;
	void processResult(const WorkResult *result, bool cancelled);
	void bindResource(const std::string &name, int id);
	EStatus generateWork(WorkUnit *unit, int worker);

	//! @}
	// ======================================================================

	MTS_DECLARE_CLASS()
protected:
	/// Virtual destructor
	virtual ~BlockedWLRProcess();
protected:
	ref<RenderQueue> m_queue;
	ref<Scene> m_scene;
	ref<Film> m_film;
	ref<Film> m_film_depth;
	const RenderJob *m_parent;
	int m_resultCount;
	ref<Mutex> m_resultMutex;
	ProgressReporter *m_progress;
	int m_borderSize;
	Bitmap::EPixelFormat m_pixelFormat;
	int m_channelCount;
	bool m_warnInvalid;
};

MTS_NAMESPACE_END

#endif /* __MITSUBA_RENDER_WLRPROC_H_ */