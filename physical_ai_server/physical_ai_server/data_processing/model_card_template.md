---
# For reference on model card metadata, see the spec: https://github.com/huggingface/hub-docs/blob/main/modelcard.md?plain=1
# Doc / guide: https://huggingface.co/docs/hub/model-cards
# prettier-ignore
{{card_data}}
---

# Model Card for {{ model_name | default("Model", true) }}

This model was trained using [Physical AI Tools](https://github.com/ROBOTIS-GIT/physical_ai_tools) and [LeRobot](https://github.com/huggingface/lerobot).

## Model Description

{{ model_description | default("", true) }}

## Citation

**BibTeX:**

```bibtex
{{ citation_bibtex | default("[More Information Needed]", true) }}
```
