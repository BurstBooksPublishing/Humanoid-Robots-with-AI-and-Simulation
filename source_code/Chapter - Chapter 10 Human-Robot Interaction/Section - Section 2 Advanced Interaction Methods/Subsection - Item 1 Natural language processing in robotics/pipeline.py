import numpy as np
from transformers import AutoTokenizer, AutoModelForSequenceClassification
# ROS2 imports omitted for brevity; assume subscription to /asr_hypotheses topic

tokenizer = AutoTokenizer.from_pretrained("distilbert-base-uncased")
model = AutoModelForSequenceClassification.from_pretrained("distilbert-base-uncased", num_labels=5)
# simple label list: ['goto','pick','stop','status','fallback']

def intent_probs_from_text(text):
    # tokenization and forward pass (batch size 1)
    inputs = tokenizer(text, return_tensors="pt")
    logits = model(**inputs).logits.detach().numpy().ravel()
    probs = np.exp(logits) / np.sum(np.exp(logits))
    return probs  # array of P(intent|hypothesis)

def aggregate_intent(asr_hyps):
    # asr_hyps: list of (hypothesis_text, p_hyp) from ASR lattice
    agg = np.zeros(5)
    for h_text, p_h in asr_hyps:
        agg += intent_probs_from_text(h_text) * p_h  # Eq. (1)
    return agg / np.sum(agg)

# simple POMDP belief state for intents
belief = np.ones(5) / 5  # uniform prior

def belief_update(belief, obs_probs, trans_matrix, obs_model_diag):
    # obs_probs: P(obs|s') approximated by obs likelihood per intent
    # trans_matrix: P(s'|s,a) simplified here
    pred = trans_matrix.T.dot(belief)  # predict step
    new_b = obs_probs * pred           # Bayes correction
    return new_b / np.sum(new_b)      # normalize

# Example runtime: subscribe to ASR hyp list, call aggregate_intent, update belief