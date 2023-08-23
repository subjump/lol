/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
BDprojektasRJAudioProcessor::BDprojektasRJAudioProcessor(){
#if JucePlugin_PreferredChannelConfigurations
     : AudioProcessor (BusesProperties()
                     #if ! JucePlugin_IsMidiEffect
                      #if ! JucePlugin_IsSynth
                       .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                      #endif
                       .withOutput ("Output", juce::AudioChannelSet::stereo(), true)
                     #endif
                       )
#endif
}

BDprojektasRJAudioProcessor::~BDprojektasRJAudioProcessor()
{
}

//==============================================================================
const juce::String BDprojektasRJAudioProcessor::getName() const
{
    return JucePlugin_Name;
}

bool BDprojektasRJAudioProcessor::acceptsMidi() const
{
   #if JucePlugin_WantsMidiInput
    return true;
   #else
    return false;
   #endif
}

bool BDprojektasRJAudioProcessor::producesMidi() const
{
   #if JucePlugin_ProducesMidiOutput
    return true;
   #else
    return false;
   #endif
}

bool BDprojektasRJAudioProcessor::isMidiEffect() const
{
   #if JucePlugin_IsMidiEffect
    return true;
   #else
    return false;
   #endif
}

double BDprojektasRJAudioProcessor::getTailLengthSeconds() const
{
    return 0.0;
}

int BDprojektasRJAudioProcessor::getNumPrograms()
{
    return 1;   // NB: some hosts don't cope very well if you tell them there are 0 programs,
                // so this should be at least 1, even if you're not really implementing programs.
}

int BDprojektasRJAudioProcessor::getCurrentProgram()
{
    return 0;
}

void BDprojektasRJAudioProcessor::setCurrentProgram (int index)
{
}

const juce::String BDprojektasRJAudioProcessor::getProgramName (int index)
{
    return {};
}

void BDprojektasRJAudioProcessor::changeProgramName (int index, const juce::String& newName)
{
}

//==============================================================================
void BDprojektasRJAudioProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    // Use this method as the place to do any pre-playback
    // initialisation that you need..
    
    //filtru paruosimas pries ju panaudojima
    juce::dsp::ProcessSpec spec;
    
    spec.maximumBlockSize = samplesPerBlock; //max imties sk.
    spec.numChannels = 1; //kanalu sk.
    spec.sampleRate = sampleRate; //diskretizavimo dažnis
    
    leftChain.prepare(spec);
    rightChain.prepare(spec);
    
    updateFilters();
    
    leftChannelFifo.prepare(samplesPerBlock);
    rightChannelFifo.prepare(samplesPerBlock);
    
    osc.initialise([](float x) { return std::sin(x); });
    
    spec.numChannels = getTotalNumOutputChannels();
    osc.prepare(spec);
    osc.setFrequency(100);
}

void BDprojektasRJAudioProcessor::releaseResources()
{
    // When playback stops, you can use this as an opportunity to free up any
    // spare memory, etc.
}

#ifndef JucePlugin_PreferredChannelConfigurations
bool BDprojektasRJAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
  #if JucePlugin_IsMidiEffect
    juce::ignoreUnused (layouts);
    return true;
  #else
    // This is the place where you check if the layout is supported.
    // In this template code we only support mono or stereo.
    // Some plugin hosts, such as certain GarageBand versions, will only
    // load plugins that support stereo bus layouts.
    if (layouts.getMainOutputChannelSet() != juce::AudioChannelSet::mono()
     && layouts.getMainOutputChannelSet() != juce::AudioChannelSet::stereo())
        return false;

    // This checks if the input layout matches the output layout
   #if ! JucePlugin_IsSynth
    if (layouts.getMainOutputChannelSet() != layouts.getMainInputChannelSet())
        return false;
   #endif

    return true;
  #endif
}
#endif

void BDprojektasRJAudioProcessor::processBlock (juce::AudioBuffer<float>& buffer, juce::MidiBuffer& midiMessages)
{
    juce::ScopedNoDenormals noDenormals;
    auto totalNumInputChannels  = getTotalNumInputChannels();
    auto totalNumOutputChannels = getTotalNumOutputChannels();

    // In case we have more outputs than inputs, this code clears any output
    // channels that didn't contain input data, (because these aren't
    // guaranteed to be empty - they may contain garbage).
    // This is here to avoid people getting screaming feedback
    // when they first compile a plugin, but obviously you don't need to keep
    // this code if your algorithm always overwrites all the output channels.
    for (auto i = totalNumInputChannels; i < totalNumOutputChannels; ++i)
        buffer.clear (i, 0, buffer.getNumSamples());
    
    updateFilters();

    juce::dsp::AudioBlock<float> block(buffer); //garso blokas, kuriame pateikiamas buferis su kanalais
    
    /*
      ==============================================================================

        THIS IS FOR SINEWAVE DEBUGGING
     
     buffer.clear();
     juce::dsp::ProcessContextReplacing<float> stereoContext(block);
     osc.process(stereoContext);

      ==============================================================================
    */
    
    auto leftBlock = block.getSingleChannelBlock(0);
    auto rightBlock = block.getSingleChannelBlock(1);
    
    juce::dsp::ProcessContextReplacing<float> leftContext(leftBlock);
    juce::dsp::ProcessContextReplacing<float> rightContext(rightBlock);
    
    leftChain.process(leftContext);
    rightChain.process(rightContext);
    
    leftChannelFifo.update(buffer);
    rightChannelFifo.update(buffer);
}

//==============================================================================
bool BDprojektasRJAudioProcessor::hasEditor() const
{
    return true; // (change this to false if you choose to not supply an editor)
}

juce::AudioProcessorEditor* BDprojektasRJAudioProcessor::createEditor()
{
    return new BDprojektasRJAudioProcessorEditor (*this);
//    return new juce::GenericAudioProcessorEditor(*this);
}

//==============================================================================
void BDprojektasRJAudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    // You should use this method to store your parameters in the memory block.
    // You could do that either as raw data, or use the XML or ValueTree classes
    // as intermediaries to make it easy to save and load complex data.
    
    //perkrovus iskiepi reiksmes issaugomos !nunulinamos
    juce::MemoryOutputStream mos(destData, true);
    apvts.state.writeToStream(mos);
}

void BDprojektasRJAudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    // You should use this method to restore your parameters from this memory block,
    // whose contents will have been created by the getStateInformation() call.
    auto tree = juce::ValueTree::readFromData(data, sizeInBytes);
    if( tree.isValid() )
    {
        apvts.replaceState(tree);
        updateFilters();
    }
}

ChainSettings getChainSettings(juce::AudioProcessorValueTreeState& apvts) //gauname reiksmes is apvts
{
    ChainSettings settings;
    
    settings.lowCutFreq = apvts.getRawParameterValue("LowCut Freq")->load();//raw grazina reiksmes is nustatyto diapazono parametruose
    settings.highCutFreq = apvts.getRawParameterValue("HighCut Freq")->load();
    settings.peakFreq = apvts.getRawParameterValue("Peak Freq")->load();
    settings.peakGainInDecibels = apvts.getRawParameterValue("Peak Gain")->load();
    settings.peakQuality = apvts.getRawParameterValue("Peak Quality")->load();
    settings.lowCutSlope = static_cast<Slope>(apvts.getRawParameterValue("LowCut Slope")->load());
    settings.highCutSlope = static_cast<Slope>(apvts.getRawParameterValue("HighCut Slope")->load());
    
    settings.lowCutBypassed = apvts.getRawParameterValue("LowCut Bypassed")->load() > 0.5f;
    settings.peakBypassed = apvts.getRawParameterValue("Peak Bypassed")->load() > 0.5f;
    settings.highCutBypassed = apvts.getRawParameterValue("HighCut Bypassed")->load() > 0.5f;

    settings.reverbSize = apvts.getRawParameterValue("Reverb Size")->load();
    settings.reverbDamping = apvts.getRawParameterValue("Reverb Damping")->load();
    settings.reverbWet = apvts.getRawParameterValue("Reverb Dry/Wet")->load();
    settings.delayTime = apvts.getRawParameterValue("Delay Time")->load();
    settings.delayFeedBack = apvts.getRawParameterValue("Delay Feedback")->load();
    settings.delayWet = apvts.getRawParameterValue("Delay Dry/Wet")->load();
    
    return settings;
}

Coefficients makePeakFilter(const ChainSettings& chainSettings, double sampleRate)
{
    return juce::dsp::IIR::Coefficients<float>::makePeakFilter(sampleRate, chainSettings.peakFreq, chainSettings.peakQuality, juce::Decibels::decibelsToGain(chainSettings.peakGainInDecibels));
}

void BDprojektasRJAudioProcessor::updatePeakFilter(const ChainSettings &chainSettings)
{
//    auto peakCoefficients = juce::dsp::IIR::Coefficients<float>::makePeakFilter(getSampleRate(), chainSettings.peakFreq, chainSettings.peakQuality, juce::Decibels::decibelsToGain(chainSettings.peakGainInDecibels));
    
    auto peakCoefficients = makePeakFilter(chainSettings, getSampleRate());
    
    leftChain.setBypassed<ChainPositions::Peak>(chainSettings.peakBypassed);
    rightChain.setBypassed<ChainPositions::Peak>(chainSettings.peakBypassed);
    
    updateCoefficients(leftChain.get<ChainPositions::Peak>().coefficients, peakCoefficients);
    updateCoefficients(rightChain.get<ChainPositions::Peak>().coefficients, peakCoefficients);
}

void updateCoefficients(Coefficients &old, const Coefficients &replacements)
{
    *old = *replacements;
}

void BDprojektasRJAudioProcessor::updateLowCutFilters(const ChainSettings &chainSettings)
{
    auto cutCoefficients = makeLowCutFilter(chainSettings, getSampleRate());
    
    auto& leftLowCut = leftChain.get<ChainPositions::LowCut>();
    auto& rightLowCut = rightChain.get<ChainPositions::LowCut>();
    
    leftChain.setBypassed<ChainPositions::LowCut>(chainSettings.lowCutBypassed);
    rightChain.setBypassed<ChainPositions::LowCut>(chainSettings.lowCutBypassed);
    
    updateCutFilter(leftLowCut, cutCoefficients, chainSettings.lowCutSlope);
    updateCutFilter(rightLowCut, cutCoefficients, chainSettings.lowCutSlope);
}

void BDprojektasRJAudioProcessor::updateHighCutFilters(const ChainSettings &chainSettings)
{
    auto highCutCoefficients = makeHighCutFilter(chainSettings, getSampleRate());
    
    auto& leftHighCut = leftChain.get<ChainPositions::HighCut>();
    auto& rightHighCut = rightChain.get<ChainPositions::HighCut>();
    
    leftChain.setBypassed<ChainPositions::HighCut>(chainSettings.highCutBypassed);
    rightChain.setBypassed<ChainPositions::HighCut>(chainSettings.highCutBypassed);
    
    updateCutFilter(leftHighCut, highCutCoefficients, chainSettings.highCutSlope);
    updateCutFilter(rightHighCut, highCutCoefficients, chainSettings.highCutSlope);
}

void BDprojektasRJAudioProcessor::updateDelay(ChainSettings &chainSettings)
{

    auto& leftDelay = leftChain.get<ChainPositions::delay>();
    auto& rightDelay = rightChain.get<ChainPositions::delay>();

    leftDelay.setDelayTime(0, chainSettings.delayTime);
    leftDelay.setDelayTime(1, chainSettings.delayTime);
    leftDelay.setFeedback(chainSettings.delayFeedBack);
    leftDelay.setWetLevel(chainSettings.delayWet);

    rightDelay.setDelayTime(0, chainSettings.delayTime);
    rightDelay.setDelayTime(1, chainSettings.delayTime);
    rightDelay.setFeedback(chainSettings.delayFeedBack);
    rightDelay.setWetLevel(chainSettings.delayWet);

}

void BDprojektasRJAudioProcessor::updateReverb(ChainSettings &chainSettings)
{
    auto& leftReverb = leftChain.get<ChainPositions::reverb>();
    auto& rightReverb = rightChain.get<ChainPositions::reverb>();

    auto parameters = leftReverb.getParameters();
    parameters.wetLevel = chainSettings.reverbWet;
    parameters.damping = chainSettings.reverbDamping;
    parameters.dryLevel = 1.f - parameters.wetLevel;
    parameters.roomSize = chainSettings.reverbSize;

    leftReverb.setParameters(parameters);
    rightReverb.setParameters(parameters);
}

void BDprojektasRJAudioProcessor::updateFilters()
{
    auto chainSettings = getChainSettings(apvts);
    
    updateLowCutFilters(chainSettings);
    updatePeakFilter(chainSettings);
    updateHighCutFilters(chainSettings);
    updateDelay(chainSettings);
    updateReverb(chainSettings);
}

//Audio parametru kurimas
juce::AudioProcessorValueTreeState::ParameterLayout BDprojektasRJAudioProcessor::createParameterLayout()
{
    juce::AudioProcessorValueTreeState::ParameterLayout layout;
    
    layout.add(std::make_unique<juce::AudioParameterFloat>(juce::ParameterID("LowCut Freq", 1),
                                                           "LowCut Freq",
                                                           juce::NormalisableRange<float>(20.f, 20000.f, 1.f, 0.25f),
                                                           20.f));

    layout.add(std::make_unique<juce::AudioParameterFloat>(juce::ParameterID("HighCut Freq", 1),
                                                           "HighCut Freq",
                                                           juce::NormalisableRange<float>(20.f, 20000.f, 1.f, 0.25f),
                                                           20000.f));

    layout.add(std::make_unique<juce::AudioParameterFloat>(juce::ParameterID("Peak Freq", 1),
                                                           "Peak Freq",
                                                           juce::NormalisableRange<float>(20.f, 20000.f, 1.f, 0.25f),
                                                           750.f));

    layout.add(std::make_unique<juce::AudioParameterFloat>(juce::ParameterID("Peak Gain", 1),
                                                           "Peak Gain",
                                                           juce::NormalisableRange<float>(-24.f, 24.f, 0.5f, 1.f),
                                                           0.0f));

    layout.add(std::make_unique<juce::AudioParameterFloat>(juce::ParameterID("Peak Quality", 1),
                                                           "Peak Quality",
                                                           juce::NormalisableRange<float>(0.1f, 10.f, 0.05f, 1.f),
                                                           1.f));

    layout.add(std::make_unique<juce::AudioParameterFloat>(juce::ParameterID("Reverb Size", 1),
                                                           "Reverb Size", juce::NormalisableRange<float>(0.f, 1.f, 0.01f, 1.f),
                                                            0.0f));

    layout.add(std::make_unique<juce::AudioParameterFloat>(juce::ParameterID("Reverb Damping", 1),
                                                           "Reverb Damping",
                                                           juce::NormalisableRange<float>(0.f, 1.f, 0.01f, 1.f),
                                                           0.0f));

    layout.add(std::make_unique<juce::AudioParameterFloat>(juce::ParameterID("Reverb Dry/Wet", 1),
                                                           "Reverb Dry/Wet",
                                                           juce::NormalisableRange<float>(0.f, 1.f, 0.01f, 1.f),
                                                           0.0f));

    layout.add(std::make_unique<juce::AudioParameterFloat>(juce::ParameterID("Delay Time", 1),
                                                           "Delay Time", juce::NormalisableRange<float>(0.f, 2.f, 0.01f, 1.f),
                                                           0.0f));

    layout.add(std::make_unique<juce::AudioParameterFloat>(juce::ParameterID("Delay Feedback", 1),
                                                           "DelayFeedback",
                                                           juce::NormalisableRange<float>(0.f, 1.f, 0.01f, 1.f),
                                                           0.0f));

    layout.add(std::make_unique<juce::AudioParameterFloat>(juce::ParameterID("Delay Dry/Wet", 1),
                                                           "Delay Dry/Wet",
                                                           juce::NormalisableRange<float>(0.f, 1.f, 0.01f, 1.f),
                                                           0.0f));
    
    juce::StringArray stringArray;
    for( int i = 0; i < 4; ++i )
    {
        juce::String str;
        str << (12 + i*12);
        str << " db/Oct";
        stringArray.add(str);
    }
    
    layout.add(std::make_unique<juce::AudioParameterChoice>(juce::ParameterID("LowCut Slope", 1), "LowCut Slope", stringArray, 0));
    layout.add(std::make_unique<juce::AudioParameterChoice>(juce::ParameterID("HighCut Slope", 1), "HighCut Slope", stringArray, 0));
    
    layout.add(std::make_unique<juce::AudioParameterBool>(juce::ParameterID("LowCut Bypassed", 1), "LowCut Bypassed", false));
    layout.add(std::make_unique<juce::AudioParameterBool>(juce::ParameterID("Peak Bypassed", 1), "Peak Bypassed", false));
    layout.add(std::make_unique<juce::AudioParameterBool>(juce::ParameterID("HighCut Bypassed", 1), "HighCut Bypassed", false));
    layout.add(std::make_unique<juce::AudioParameterBool>(juce::ParameterID("Analyzer Enabled", 1), "Analyzer Enabled", true));
    
    return layout;
}

//==============================================================================
// This creates new instances of the plugin..
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new BDprojektasRJAudioProcessor();
}
