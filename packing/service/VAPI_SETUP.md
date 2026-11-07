# Vapi setup guide — voice ordering for the packing robot

**Audience:** whoever configures the Vapi side. You don't need to touch the robot or the Python code.
Everything Vapi needs from our server is in this file.

## What we're building

1. A customer **calls our Vapi phone number**. The **Ordering assistant** answers, tells them what's
   available, takes their order, and confirms it.
2. The assistant places the order by calling **our server** (Vapi "tools" / function calls).
   Our server queues it; a robot arm packs orders one at a time.
3. When the order is packed, **our server asks Vapi to call the customer back** using a second
   assistant, the **Callback assistant**, which tells them their order is ready.

```
Customer --call--> Ordering assistant --tool calls--> our server (queue) --> robot packs
Customer <--call-- Callback assistant <--POST /call (Vapi API)-- our server
```

## What you need to create in Vapi

| Thing | Notes |
|---|---|
| **Phone number** | Inbound for customers. It must also be able to place **outbound** calls (free Vapi numbers can't — import a Twilio/Vonage/Telnyx number). |
| **Credential** for our server | Custom credential, type **Bearer Token** (or header `X-Vapi-Secret`, no Bearer prefix). The secret value is agreed with us (`VAPI_SERVER_SECRET`). |
| **4 tools** | Definitions below. Attach the credential to their server URL. |
| **Ordering assistant** | Has the 4 tools. Attached to the phone number for inbound calls. |
| **Callback assistant** | No tools needed. Used only for the outbound "your order is ready" call. |
| **Private API key** | For our server to place callback calls. |

## What to send back to us

Put these in `packing/service/.env` on the server (or send them to whoever runs it):

```
VAPI_API_KEY=               # private API key
VAPI_PHONE_NUMBER_ID=       # id of the phone number callbacks are made from
VAPI_CALLBACK_ASSISTANT_ID= # id of the Callback assistant
VAPI_SERVER_SECRET=         # the secret in the credential you created
```

We'll give you the **server URL**: `https://<our-host>/vapi/webhook`
(we expose it with a tunnel such as cloudflared or ngrok; the host may change while we're developing).

## Tools

Create these 4 tools (dashboard: Tools > Create Tool > Custom/Function, or via the API) with server URL
`https://<our-host>/vapi/webhook`, and attach the credential. Exact definitions:

```json
[
  {
    "type": "function",
    "server": {
      "url": "https://YOUR-SERVER-HOST/vapi/webhook"
    },
    "function": {
      "name": "list_products",
      "description": "List the products that can be ordered and how many are in stock.",
      "parameters": {
        "type": "object",
        "properties": {}
      }
    }
  },
  {
    "type": "function",
    "server": {
      "url": "https://YOUR-SERVER-HOST/vapi/webhook"
    },
    "function": {
      "name": "place_order",
      "description": "Place an order to be packed. The customer gets a phone call back when it is ready. Confirm the items and the callback number with the customer before calling this.",
      "parameters": {
        "type": "object",
        "properties": {
          "customer_name": {
            "type": "string",
            "description": "Customer's name"
          },
          "phone": {
            "type": "string",
            "description": "Callback number. Omit to use the number they called from."
          },
          "items": {
            "type": "array",
            "description": "Products and quantities",
            "items": {
              "type": "object",
              "properties": {
                "product": {
                  "type": "string",
                  "description": "Product name or SKU from list_products"
                },
                "quantity": {
                  "type": "integer",
                  "minimum": 1
                }
              },
              "required": [
                "product"
              ]
            }
          }
        },
        "required": [
          "items"
        ]
      }
    }
  },
  {
    "type": "function",
    "server": {
      "url": "https://YOUR-SERVER-HOST/vapi/webhook"
    },
    "function": {
      "name": "check_order",
      "description": "Get the status of an order. Without order_id, returns the caller's latest order.",
      "parameters": {
        "type": "object",
        "properties": {
          "order_id": {
            "type": "integer"
          }
        }
      }
    }
  },
  {
    "type": "function",
    "server": {
      "url": "https://YOUR-SERVER-HOST/vapi/webhook"
    },
    "function": {
      "name": "cancel_order",
      "description": "Cancel an order that hasn't started packing yet.",
      "parameters": {
        "type": "object",
        "properties": {
          "order_id": {
            "type": "integer"
          }
        },
        "required": [
          "order_id"
        ]
      }
    }
  }
]
```

(Regenerate with the real URL any time: `python -m packing.service tools --url https://<our-host>/vapi/webhook`.)

**How they behave**

| Tool | Arguments | Returns (a sentence to say to the caller) |
|---|---|---|
| `list_products` | none | "We have: widget (3 in stock), gadget (1 in stock). One box holds up to 2 items." |
| `place_order` | `items` (required) `[{product, quantity}]`; `customer_name`; `phone` (only if different from the number they're calling from) | "Order number 12 is placed: 1 widget, 1 gadget. You're number 3 in line. We'll call you at +15551234567 as soon as it's packed." |
| `check_order` | `order_id` (optional; without it, the caller's latest order) | "Order 12 (1 widget, 1 gadget) is being packed right now." |
| `cancel_order` | `order_id` | "Order 12 is cancelled." (Only possible before packing starts.) |

- `product` can be the product name ("widget") or its SKU ("A").
- Problems come back as **normal results** the assistant should read out, e.g.
  "Sorry, not enough stock: gadget (only 1 left).", "One box holds up to 2 items; that order has 3.",
  "Sorry, we don't carry 'sprocket'. We have: widget, gadget." The assistant should then help the caller adjust.
- If something breaks on our side the result has an `error` field instead; the assistant should apologise
  and offer to take their number for a follow-up.
- Calling `place_order` twice with the same tool call (Vapi retry) does **not** create a second order.

## Ordering assistant — suggested prompt

```
You are the phone ordering assistant for [BUSINESS NAME]. Orders are packed by a robot and the
customer gets a phone call back when their order is ready.

1. Greet the caller. If they ask what's available, or you need to check, call list_products.
2. Collect what they want and how many of each. One box holds a limited number of items
   (list_products tells you how many); if they want more, explain they'll need separate orders.
3. Ask for their name.
4. Read the order back and confirm. Confirm we'll call back on the number they're calling from,
   unless they give a different one (then pass it as `phone`).
5. Call place_order. Read the result to them: order number, place in line, callback number.
6. If they ask about an existing order, call check_order. To cancel, call cancel_order with the number.
Keep it short and friendly. Never invent products, stock, or order numbers — only use tool results.
```

## Callback assistant — suggested setup

Our server starts this call with these **variables** (Vapi `assistantOverrides.variableValues`), usable as
`{{...}}` in the first message and prompt:

| Variable | Example |
|---|---|
| `{{customer_name}}` | `Sam` (may be empty) |
| `{{order_id}}` | `12` |
| `{{items}}` | `1 widget, 1 gadget` |
| `{{box_id}}` | `BOX-0007` (internal, usually not spoken) |

First message: `Hi {{customer_name}}, this is [BUSINESS NAME]. Your order number {{order_id}} — {{items}} — is packed and ready.`
Prompt: answer brief questions about pickup ([PICKUP DETAILS]), then end the call. If they have a
problem with the order, apologise and say someone will follow up.

## The exact HTTP contract (for debugging)

**Vapi -> us** (`POST /vapi/webhook`, header `Authorization: Bearer <secret>` or `X-Vapi-Secret: <secret>`):
```json
{"message": {"type": "tool-calls",
  "call": {"id": "call-uuid", "customer": {"number": "+15551234567"}},
  "toolCallList": [{"id": "toolu_123", "type": "function",
    "function": {"name": "place_order",
                 "arguments": {"customer_name": "Sam", "items": [{"product": "widget", "quantity": 1}]}}}]}}
```
**us -> Vapi** (always HTTP 200; 401 only if the secret is wrong):
```json
{"results": [{"toolCallId": "toolu_123", "result": "Order number 1 is placed: 1 widget. ..."}]}
```
Other server messages (status-update, end-of-call-report, ...) are accepted and ignored for now, so
it's fine to point the assistant's general Server URL at the same endpoint.

**us -> Vapi callback** (`POST https://api.vapi.ai/call`, `Authorization: Bearer <VAPI_API_KEY>`):
```json
{"assistantId": "<VAPI_CALLBACK_ASSISTANT_ID>", "phoneNumberId": "<VAPI_PHONE_NUMBER_ID>",
 "customer": {"number": "+15551234567", "name": "Sam"},
 "assistantOverrides": {"variableValues": {"order_id": "1", "customer_name": "Sam",
                                            "items": "1 widget", "box_id": "BOX-0001"}}}
```

## Testing without a phone

From any machine that can reach the server:
```bash
curl -X POST https://<our-host>/vapi/webhook \
  -H "Authorization: Bearer <secret>" -H "Content-Type: application/json" \
  -d '{"message":{"type":"tool-calls","call":{"id":"t","customer":{"number":"+15551234567"}},"toolCallList":[{"id":"x1","type":"function","function":{"name":"list_products","arguments":{}}}]}}'
```
The Vapi dashboard's tool **Test** button also works (it sends the tool's headers but not a saved
credential, so ask us to temporarily disable the secret, or add the header to the tool).

## Things to confirm on the first real call
- Caller number: we read `message.call.customer.number` (fallback `message.customer.number`). If
  `place_order` replies "I need a phone number to call you back", that field is missing — tell us.
- Phone numbers are normalised to E.164; 10-digit numbers are assumed US (+1).
- Product names/SKUs are placeholders (widget/gadget) until the real catalog is set.
