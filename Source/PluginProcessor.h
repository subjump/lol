/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#pragma once

#include <JuceHeader.h>

#include <array>
template<typename T>
struct Fifo
{
    void prepare(int numChannels, int numSamples)
    {
        static_assert( std::is_same_v<T, juce::AudioBuffer<float>>,
                      "prepare(numChannels, numSamples) should only be used when the Fifo is holding juce::AudioBuffer<float>");
        for( auto& buffer : buffers)
        {
            buffer.setSize(numChannels,
                           numSamples,
                           false,
                           true,
                           true);
            buffer.clear();
        }
    }
    
    void prepare(size_t numElements)
    {
        static_assert( std::is_same_v<T, std::vector<float>>,
                      "prepare(numElements) should only be used when the Fifo is holding std::vector<float>");
        for( auto& buffer : buffers )
        {
            buffer.clear();
            buffer.resize(numElements, 0);
        }
    }
    
    bool push(const T& t)
    {
        auto write = fifo.write(1);
        if( write.blockSize1 > 0 )
        {
            buffers[write.startIndex1] = t;
            return true;
        }
        
        return false;
    }
    
    bool pull(T& t)
    {
        auto read = fifo.read(1);
        if( read.blockSize1 > 0 )
        {
            t = buffers[read.startIndex1];
            return true;
        }
        
        return false;
    }
    
    int getNumAvailableForReading() const
    {
        return fifo.getNumReady();
    }
private:
    static constexpr int Capacity = 30;
    std::array<T, Capacity> buffers;
    juce::AbstractFifo fifo {Capacity};
};

template <typename Type>
class DelayLine
{
public:
    void clear() noexcept
    {
        std::fill(rawData.begin, rawData.end(), Type(0));
    }

    size_t size() const noexcept
    {
        return rawData.size();
    }

    void resize(size_t newValue)
    {
        rawData.resize(newValue);
        leastRecentIndex = 0;
    }

    Type back() const noexcept
    {
        return rawData[leastRecentIndex];
    }

    Type get(size_t delayInSamples) const noexcept
    {
        jassert(delayInSamples >= 0 && delayInSamples < size());
        return rawData[(leastRecentIndex + 1 + delayInSamples) % size()];
    }

    Type set(size_t delayInSamples, Type newValue) noexcept
    {
        jassert(delayInSamples >= 0 && delayInSamples < size());
        rawData[(leastRecentIndex + 1 + delayInSamples) % size()] = newValue;
    }

    void push(Type valueToAdd) noexcept
    {
        rawData[leastRecentIndex] = valueToAdd;
        leastRecentIndex = leastRecentIndex == 0 ? size() - 1 : leastRecentIndex - 1;
    }

private:
    std::vector<Type> rawData;
    size_t leastRecentIndex = 0;
};


template <typename Type, size_t maxNumChannels = 2>
class Delay
{
public:
    //==============================================================================
    Delay()
    {
        setMaxDelayTime(2.1f);
        setDelayTime(0, 0.7f);
        setDelayTime(1, 0.5f);
        setWetLevel(0.5f);
        setFeedback(0.5f);
    }

    //==============================================================================
    void prepare(const juce::dsp::ProcessSpec& spec)
    {
        jassert(spec.numChannels <= maxNumChannels);
        sampleRate = (Type)spec.sampleRate;
        updateDelayLineSize();
        updateDelayTime();

        filterCoefs = juce::dsp::IIR::Coefficients<Type>::makeFirstOrderHighPass(sampleRate, Type(1e3));

        for (auto& f : filters)
        {
            f.prepare(spec);
            f.coefficients = filterCoefs;
        }
    }

    //==============================================================================
    void reset() noexcept
    {
        for (auto& f : filters)
            filters.reset();

        for (auto& dline : delayLines)
            dline.clear();
    }

    //==============================================================================
    size_t getNumChannels() const noexcept
    {
        return delayLines.size();
    }

    //==============================================================================
    void setMaxDelayTime(Type newValue)
    {
        jassert(newValue > Type(0));
        maxDelayTime = newValue;
        updateDelayLineSize();
    }

    //==============================================================================
    void setFeedback(Type newValue) noexcept
    {
        jassert(newValue >= Type(0) && newValue <= Type(1));
        feedback = newValue;
    }

    //==============================================================================
    void setWetLevel(Type newValue) noexcept
    {
        jassert(newValue >= Type(0) && newValue <= Type(1));
        wetLevel = newValue;
    }

    //==============================================================================
    void setDelayTime(size_t channel, Type newValue)
    {
        if (channel >= getNumChannels())
        {
            jassertfalse;
            return;
        }

        jassert(newValue >= Type(0));
        delayTimes[channel] = newValue;

        updateDelayTime();
    }

    //==============================================================================
    template <typename ProcessContext>
    void process(const ProcessContext& context) noexcept
    {
        auto& inputBlock = context.getInputBlock();
        auto& outputBlock = context.getOutputBlock();
        auto numSamples = outputBlock.getNumSamples();
        auto numChannels = outputBlock.getNumChannels();

        jassert(inputBlock.getNumSamples() == numSamples);
        jassert(inputBlock.getNumChannels() == numChannels);

        juce::ignoreUnused(numSamples);

        for (size_t ch = 0; ch < numChannels; ++ch)
        {
            auto* input = inputBlock.getChannelPointer(ch);
            auto* output = outputBlock.getChannelPointer(ch);
            auto& dline = delayLines[ch];
            auto delayTime = delayTimesSample[ch];
            auto& filter = filters[ch];

            for (size_t i = 0; i < numSamples; ++i)
            {
                auto delayedSample = filter.processSample(dline.get(delayTime));
                auto inputSample = input[i];
                auto dlineInputSample = std::tanh(inputSample + feedback * delayedSample);
                dline.push(dlineInputSample);
                auto outputSample = inputSample + wetLevel * delayedSample;
                output[i] = outputSample;
            }
        }
    }

private:
    std::array<DelayLine<Type>, maxNumChannels> delayLines;
    std::array<size_t, maxNumChannels> delayTimesSample;
    std::array<Type, maxNumChannels> delayTimes;
    Type feedback{ Type(0) };
    Type wetLevel{ Type(0) };

    std::array<juce::dsp::IIR::Filter<Type>, maxNumChannels> filters;
    typename juce::dsp::IIR::Coefficients<Type>::Ptr filterCoefs;

    Type sampleRate{ Type(44.1e3) };
    Type maxDelayTime{ Type(2) };

    //==============================================================================
    void updateDelayLineSize()
    {
        auto delayLineSizeSamples = (size_t)std::ceil(maxDelayTime * sampleRate);

        for (auto& dline : delayLines)
        {
            dline.resize(delayLineSizeSamples);
        }
    }

    //==============================================================================
    void updateDelayTime() noexcept
    {
        for (size_t ch = 0; ch < maxNumChannels; ++ch)
            delayTimesSample[ch] = (size_t)juce::roundToInt(delayTimes[ch] * sampleRate);
    }
};

enum Channel
{
    Right, // effectively 0
    Left // effectively 1
};

template<typename BlockType>
struct SingleChannelSampleFifo
{
  SingleChannelSampleFifo(Channel ch) : channelToUse(ch)
    {
        prepared.set(false);
    }
    
    void update(const BlockType& buffer)
    {
        jassert(prepared.get());
        jassert(buffer.getNumChannels() > channelToUse );
        auto* channelPtr = buffer.getReadPointer(channelToUse);
        
        for( int i = 0; i < buffer.getNumSamples(); ++i )
        {
            pushNextSampleIntoFifo(channelPtr[i]);
        }
    }
    
    void prepare(int bufferSize)
    {
        prepared.set(false);
        size.set(bufferSize);
        
        bufferToFill.setSize(1,
                             bufferSize,
                             false,
                             true,
                             true);
        audioBufferFifo.prepare(1, bufferSize);
        fifoIndex = 0;
        prepared.set(true);
    }
    //==============================================================================
    int getNumCompleteBuffersAvailable() const { return audioBufferFifo.getNumAvailableForReading(); }
    bool isPrepared() const { return prepared.get(); }
    int getSize() const { return size.get(); }
    //==============================================================================
    bool getAudioBuffer(BlockType& buf) { return audioBufferFifo.pull(buf); }
private:
    Channel channelToUse;
    int fifoIndex = 0;
    Fifo<BlockType> audioBufferFifo;
    BlockType bufferToFill;
    juce::Atomic<bool> prepared = false;
    juce::Atomic<int> size = 0;
    
    void pushNextSampleIntoFifo(float sample)
    {
        if (fifoIndex == bufferToFill.getNumSamples())
        {
            auto ok = audioBufferFifo.push(bufferToFill);
            
            juce::ignoreUnused(ok);
            
            fifoIndex = 0;
        }
        
        bufferToFill.setSample(0, fifoIndex, sample);
        ++fifoIndex;
    }
};

enum Slope
{
    Slope_12,
    Slope_24,
    Slope_36,
    Slope_48
};

struct ChainSettings //strukturizuoja parametru reiksmes
{
    float peakFreq { 0 }, peakGainInDecibels{ 0 }, peakQuality {1.f};
    float lowCutFreq { 0 }, highCutFreq { 0 };
    float reverbSize{ 0.5f }, reverbDamping{ 0.5f }, reverbWet{ 0.5f };
    float delayTime{ 0.5f }, delayFeedBack{ 0.5f }, delayWet{ 0.5f };
    
    Slope lowCutSlope { Slope::Slope_12 }, highCutSlope { Slope::Slope_12 };
    
    bool lowCutBypassed { false }, peakBypassed { false }, highCutBypassed { false };
};

ChainSettings getChainSettings(juce::AudioProcessorValueTreeState& apvts);

using Filter = juce::dsp::IIR::Filter<float>;

using CutFilter = juce::dsp::ProcessorChain<Filter, Filter, Filter, Filter>;

using MonoChain = juce::dsp::ProcessorChain<CutFilter, Filter, CutFilter, juce::dsp::Reverb, Delay<float>>;

enum ChainPositions
{
    LowCut,
    Peak,
    HighCut,
    reverb,
    delay
};

using Coefficients = Filter::CoefficientsPtr;
void updateCoefficients(Coefficients& old, const Coefficients& replacements);

Coefficients makePeakFilter(const ChainSettings& chainSettings, double sampleRate);

template<int Index, typename ChainType, typename CoefficientType>
void update(ChainType& chain, const CoefficientType& coefficients)
{
    updateCoefficients(chain.template get<Index>().coefficients, coefficients[Index]);
    chain.template setBypassed<Index>(false);
}

template<typename ChainType, typename CoefficientType>
void updateCutFilter(ChainType& chain,
                     const CoefficientType& coefficients,
                     const Slope& slope)
{
    
    chain.template setBypassed<0>(true);
    chain.template setBypassed<1>(true);
    chain.template setBypassed<2>(true);
    chain.template setBypassed<3>(true);
    
    switch ( slope )
    {
        case Slope_48:
        {
            update<3>(chain, coefficients);
        }
        case Slope_36:
        {
            update<2>(chain, coefficients);
        }
        case Slope_24:
        {
            update<1>(chain, coefficients);
        }
        case Slope_12:
        {
            update<0>(chain, coefficients);
        }
    }
}

inline auto makeLowCutFilter(const ChainSettings& chainSettings, double sampleRate)
{
    return juce::dsp::FilterDesign<float>::designIIRHighpassHighOrderButterworthMethod(chainSettings.lowCutFreq, sampleRate, 2 * (chainSettings.lowCutSlope + 1));
}

inline auto makeHighCutFilter(const ChainSettings& chainSettings, double sampleRate)
{
    return juce::dsp::FilterDesign<float>::designIIRLowpassHighOrderButterworthMethod(chainSettings.highCutFreq, sampleRate, 2 * (chainSettings.highCutSlope + 1));
    
}

//==============================================================================
/**
*/
class BDprojektasRJAudioProcessor  : public juce::AudioProcessor
                            #if JucePlugin_Enable_ARA
                             , public juce::AudioProcessorARAExtension
                            #endif
{
public:
    //==============================================================================
    BDprojektasRJAudioProcessor();
    ~BDprojektasRJAudioProcessor() override;

    //==============================================================================
    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;

   #ifndef JucePlugin_PreferredChannelConfigurations
    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;
   #endif

    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    //==============================================================================
    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override;

    //==============================================================================
    const juce::String getName() const override;

    bool acceptsMidi() const override;
    bool producesMidi() const override;
    bool isMidiEffect() const override;
    double getTailLengthSeconds() const override;

    //==============================================================================
    int getNumPrograms() override;
    int getCurrentProgram() override;
    void setCurrentProgram (int index) override;
    const juce::String getProgramName (int index) override;
    void changeProgramName (int index, const juce::String& newName) override;

    //==============================================================================
    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;
    
    static juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout();
    juce::AudioProcessorValueTreeState apvts {*this, nullptr, "Parameters", createParameterLayout()}; //koordinuoja parametrus, susieja juos su GUI ir DSP

    using BlockType = juce::AudioBuffer<float>;
    SingleChannelSampleFifo<BlockType> leftChannelFifo { Channel::Left };
    SingleChannelSampleFifo<BlockType> rightChannelFifo { Channel::Right };
private:
    MonoChain leftChain, rightChain; // reikia dvieju mono egzemplioriu kad atlikti stereofonini apdorojima
    
    void updatePeakFilter(const ChainSettings& chainSettings);
    
    void updateLowCutFilters(const ChainSettings& chainSettings);
    void updateHighCutFilters(const ChainSettings& chainSettings);

    void updateDelay(ChainSettings& chainSettings);
    void updateReverb(ChainSettings& chainSettings);
    
    void updateFilters();
    
    juce::dsp::Oscillator<float> osc;
    //==============================================================================
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (BDprojektasRJAudioProcessor)
};
